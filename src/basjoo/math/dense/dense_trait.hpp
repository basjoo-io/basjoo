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

template <typename E>
concept DenseExtents = requires {
    typename E::index_type;
    E::rank();
    E::static_extent(std::size_t{0});
};

template <typename T>
concept DenseLayout = std::same_as<T, std::layout_left> || std::same_as<T, std::layout_right>;

template <typename A>
concept DenseAccessor = requires {
    typename A::element_type;
    typename A::reference;
    typename A::data_handle_type;
    typename A::offset_policy;
    {
        std::declval<const A&>().access(
            std::declval<typename A::data_handle_type>(), std::size_t{0}
        )
    } -> std::same_as<typename A::reference>;
    {
        std::declval<const A&>().offset(
            std::declval<typename A::data_handle_type>(), std::size_t{0}
        )
    } -> std::convertible_to<typename A::data_handle_type>;
};

template <
    ScalarArithmetic Scalar, std::size_t Extent = std::dynamic_extent,
    Allocatory Alloc = ::basjoo::common::AlignedAllocator<Scalar, 32>>
class Vector;

template <
    ScalarArithmetic Scalar, DenseExtents Extents = std::dextents<std::size_t, 2>,
    DenseLayout Layout = std::layout_left,
    DenseAccessor AccessorPolicy = std::default_accessor<Scalar>,
    Allocatory Alloc = ::basjoo::common::AlignedAllocator<Scalar, 32>>
class Matrix;

namespace pmr {

template <ScalarArithmetic Scalar>
using Vector =
    ::basjoo::math::Vector<Scalar, std::dynamic_extent, std::pmr::polymorphic_allocator<Scalar>>;

template <
    ScalarArithmetic Scalar, DenseLayout Layout = std::layout_left,
    DenseAccessor AccessorPolicy = std::default_accessor<Scalar>>
using Matrix = ::basjoo::math::Matrix<
    Scalar, std::dextents<std::size_t, 2>, Layout, AccessorPolicy,
    std::pmr::polymorphic_allocator<Scalar>>;

} // namespace pmr

template <typename T>
struct DenseTrait;

template <ScalarArithmetic Scalar, std::size_t Extent, Allocatory Alloc>
struct DenseTrait<Vector<Scalar, Extent, Alloc>> final {
    using element_type = Scalar;
    using value_type = std::remove_cv_t<Scalar>;
    using reference = Scalar&;
    using const_reference = const Scalar&;
    using pointer = Scalar*;
    using const_pointer = const Scalar*;
    using iterator = Scalar*;
    using const_iterator = const Scalar*;
    using reverse_iterator = std::reverse_iterator<Scalar*>;
    using const_reverse_iterator = std::reverse_iterator<const Scalar*>;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;
    using allocator_type = Alloc;
    using storage_type = std::conditional_t<
        Extent == std::dynamic_extent, std::vector<Scalar, Alloc>, std::array<Scalar, Extent>>;
    static constexpr std::size_t extent = Extent;
};

template <
    ScalarArithmetic Scalar, DenseExtents Extents, DenseLayout Layout, DenseAccessor AccessorPolicy,
    Allocatory Alloc>
struct DenseTrait<Matrix<Scalar, Extents, Layout, AccessorPolicy, Alloc>> final {
    using element_type = Scalar;
    using value_type = std::remove_cv_t<Scalar>;
    using reference = typename AccessorPolicy::reference;
    using const_reference = const reference;
    using pointer = Scalar*;
    using const_pointer = const Scalar*;
    using size_type = typename Extents::size_type;
    using difference_type = std::ptrdiff_t;
    using index_type = typename Extents::index_type;
    using rank_type = typename Extents::rank_type;
    using data_handle_type = typename AccessorPolicy::data_handle_type;
    using allocator_type = Alloc;
    using extents_type = Extents;
    using layout_type = Layout;
    using accessor_type = AccessorPolicy;
    using mapping_type = typename Layout::template mapping<Extents>;
    using storage_type = std::conditional_t<
        Extents::rank_dynamic() == 0,
        std::array<Scalar, Extents::static_extent(0) * Extents::static_extent(1)>,
        std::vector<Scalar, Alloc>>;
};

} // namespace basjoo::math
