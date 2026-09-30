/**
 * @file dok_matrix.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-24
 *
 * @copyright Copyright (c) 2024 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <utility>

#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/sparse/sparse_traits.hpp"

namespace basjoo::math {

template <ScalarArithmetic Scalar, std::integral Index, Allocatory Alloc>
class DokMatrix final {
  public:
    using value_type = typename SparseTraits<DokMatrix>::value_type;
    using index_type = typename SparseTraits<DokMatrix>::index_type;
    using reference = typename SparseTraits<DokMatrix>::reference;
    using const_reference = typename SparseTraits<DokMatrix>::const_reference;
    using size_type = typename SparseTraits<DokMatrix>::size_type;
    using difference_type = typename SparseTraits<DokMatrix>::difference_type;
    using allocator_type = typename SparseTraits<DokMatrix>::allocator_type;
    using dictionary_type = typename SparseTraits<DokMatrix>::dictionary_type;

    DokMatrix() noexcept = default;
    DokMatrix(const DokMatrix& other) = default;
    auto operator=(const DokMatrix& other) -> DokMatrix& = default;
    DokMatrix(DokMatrix&& other) noexcept = default;
    auto operator=(DokMatrix&& other) noexcept -> DokMatrix& = default;
    ~DokMatrix() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_dictionary.get_allocator();
    }

    [[using gnu: always_inline]]
    explicit DokMatrix(const allocator_type& alloc) noexcept
        : m_dictionary(alloc) {}

    [[using gnu: always_inline]]
    explicit DokMatrix(size_type nrows, size_type ncols, const allocator_type& alloc = {}) noexcept
        : m_nrows{nrows}, m_ncols{ncols}, m_dictionary(alloc) {}

    [[using gnu: always_inline]]
    DokMatrix(
        std::initializer_list<std::pair<const IndexPair<Index>, value_type>> initialize_list,
        const allocator_type& alloc = {}
    )
        : m_dictionary(std::move(initialize_list), alloc) {
        for (const auto& [index_pair, coeff] : m_dictionary) {
            m_nrows = std::max(m_nrows, static_cast<size_type>(index_pair.row + 1));
            m_ncols = std::max(m_ncols, static_cast<size_type>(index_pair.col + 1));
        }
    }

    [[using gnu: pure, always_inline, leaf]]
    auto nrows() const noexcept -> size_type {
        return m_nrows;
    }

    [[using gnu: pure, always_inline, leaf]]
    auto ncols() const noexcept -> size_type {
        return m_ncols;
    }

    [[using gnu: pure, always_inline]]
    auto nnzs() const noexcept -> size_type {
        return m_dictionary.size();
    }

    [[using gnu: always_inline]]
    auto resize(size_type nrows, size_type ncols) -> void {
#ifndef BASJOO_USE_BOOST_UNORDERED
        using std::erase_if;
#else
        using boost::unordered::erase_if;
#endif
        if (m_nrows > nrows || m_ncols > ncols) {
            erase_if(m_dictionary, [nrows, ncols](const auto& item) constexpr noexcept -> bool {
                return item.first.row >= static_cast<index_type>(nrows) ||
                       item.first.col >= static_cast<index_type>(ncols);
            });
        }
        m_nrows = nrows;
        m_ncols = ncols;
        return;
    }

    [[using gnu: always_inline]]
    auto reserve(size_type capacity) -> void {
        m_dictionary.reserve(capacity);
        return;
    }

    [[using gnu: always_inline]]
    auto clear() -> void {
        m_dictionary.clear();
        return;
    }

    [[using gnu: always_inline, leaf]]
    auto compress() -> void {
#ifndef BASJOO_USE_BOOST_UNORDERED
        using std::erase_if;
#else
        using boost::unordered::erase_if;
#endif
        erase_if(m_dictionary, [](const auto& item) constexpr noexcept -> bool {
            return item.second == static_cast<value_type>(0.0);
        });
        return;
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto coeff(size_type row, size_type col) const noexcept -> value_type {
        if (row >= m_nrows || col >= m_ncols) [[unlikely]] {
            return static_cast<value_type>(0.0);
        }
        if (const auto search =
                m_dictionary.find({static_cast<index_type>(row), static_cast<index_type>(col)});
            search != m_dictionary.cend()) {
            return search->second;
        }
        return static_cast<value_type>(0.0);
    }

    [[using gnu: flatten, leaf, hot]]
    auto updateCoeff(size_type row, size_type col, const_reference value) -> void {
        if (row >= m_nrows || col >= m_ncols) [[unlikely]] {
            return;
        }
        m_dictionary[{static_cast<index_type>(row), static_cast<index_type>(col)}] = value;
        return;
    }

    [[using gnu: flatten, leaf, hot]]
    auto eraseCoeff(size_type row, size_type col) -> void {
        if (row >= m_nrows || col >= m_ncols) [[unlikely]] {
            return;
        }
        if (auto search =
                m_dictionary.find({static_cast<index_type>(row), static_cast<index_type>(col)});
            search != m_dictionary.end()) {
            m_dictionary.erase(search);
        }
        return;
    }

    [[using gnu: pure, always_inline, hot]]
    auto operator[](size_type row, size_type col) const noexcept -> value_type {
        return coeff(row, col);
    }

    [[using gnu: pure, always_inline, leaf]]
    auto dictionary() const noexcept -> const dictionary_type& {
        return m_dictionary;
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<3>;

    size_type m_nrows;
    size_type m_ncols;
    dictionary_type m_dictionary;
};

} // namespace basjoo::math
