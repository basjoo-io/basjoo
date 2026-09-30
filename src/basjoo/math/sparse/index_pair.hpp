/**
 * @file index_pair.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-10-19
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>

#ifndef BASJOO_USE_BOOST_UNORDERED
#include <functional>
#else
#include "boost/container_hash/hash.hpp"
#endif

#include "zpp_bits.h"

namespace basjoo::math {

template <std::integral Index>
struct IndexPair final {
    using serialize = zpp::bits::members<2>;

    Index row;
    Index col;
};

template <std::integral Index>
struct IndexPairHash final {
    [[using gnu: const, always_inline, hot]]
    constexpr auto operator()(const IndexPair<Index>& index_pair) const noexcept -> std::size_t {
        std::size_t seed{0};
#ifndef BASJOO_USE_BOOST_UNORDERED
        seed = std::hash<Index>{}(index_pair.row) ^ (std::hash<Index>{}(index_pair.col) << 1);
#else
        boost::hash_combine(seed, index_pair.row);
        boost::hash_combine(seed, index_pair.col);
#endif
        return seed;
    }
};

template <std::integral Index>
struct IndexPairEqual final {
    [[using gnu: const, always_inline, leaf, hot]]
    constexpr auto operator()(
        const IndexPair<Index>& lhs, const IndexPair<Index>& rhs
    ) const noexcept -> bool {
        return lhs.row == rhs.row && lhs.col == rhs.col;
    }
};

template <std::integral Index>
struct IndexPairColumnMajorCompare final {
    [[using gnu: const, always_inline, leaf, hot]]
    constexpr auto operator()(
        const IndexPair<Index>& lhs, const IndexPair<Index>& rhs
    ) const noexcept -> bool {
        return lhs.col != rhs.col ? lhs.col < rhs.col : lhs.row < rhs.row;
    }
};

template <std::integral Index>
struct IndexPairRowMajorCompare final {
    [[using gnu: const, always_inline, leaf, hot]]
    constexpr auto operator()(
        const IndexPair<Index>& lhs, const IndexPair<Index>& rhs
    ) const noexcept -> bool {
        return lhs.row != rhs.row ? lhs.row < rhs.row : lhs.col < rhs.col;
    }
};

} // namespace basjoo::math
