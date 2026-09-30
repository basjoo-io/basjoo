/**
 * @file coo_matrix.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-10-20
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <span>
#include <vector>

#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/sparse/sparse_traits.hpp"

namespace basjoo::math {

template <ScalarArithmetic Scalar, std::integral Index, Allocatory Alloc>
class CooMatrix final {
  public:
    using value_type = typename SparseTraits<CooMatrix>::value_type;
    using index_type = typename SparseTraits<CooMatrix>::index_type;
    using reference = typename SparseTraits<CooMatrix>::reference;
    using const_reference = typename SparseTraits<CooMatrix>::const_reference;
    using size_type = typename SparseTraits<CooMatrix>::size_type;
    using difference_type = typename SparseTraits<CooMatrix>::difference_type;
    using allocator_type = typename SparseTraits<CooMatrix>::allocator_type;
    using dictionary_type = typename SparseTraits<CooMatrix>::dictionary_type;

    CooMatrix() noexcept = default;
    CooMatrix(const CooMatrix& other) = default;
    auto operator=(const CooMatrix& other) -> CooMatrix& = default;
    CooMatrix(CooMatrix&& other) noexcept = default;
    auto operator=(CooMatrix&& other) noexcept -> CooMatrix& = default;
    ~CooMatrix() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_values.get_allocator();
    }

    [[using gnu: always_inline]]
    explicit CooMatrix(const allocator_type& alloc) noexcept
        : m_offsets_map(alloc), m_values(alloc), m_row_indices(alloc), m_col_indices(alloc) {}

    [[using gnu: always_inline]]
    explicit CooMatrix(size_type nrows, size_type ncols, const allocator_type& alloc = {}) noexcept
        : m_nrows{nrows}, m_ncols{ncols}, m_offsets_map(alloc), m_values(alloc),
          m_row_indices(alloc), m_col_indices(alloc) {}

    [[using gnu: ]]
    CooMatrix(const DokMatrix<value_type, index_type, allocator_type>& dok_matrix)
        : m_nrows{dok_matrix.nrows()}, m_ncols{dok_matrix.ncols()},
          m_offsets_map(dok_matrix.nnzs(), dok_matrix.get_allocator()),
          m_values(dok_matrix.get_allocator()), m_row_indices(dok_matrix.get_allocator()),
          m_col_indices(dok_matrix.get_allocator()) {
        reserve(dok_matrix.nnzs());
        for (const auto& [index_pair, value] : dok_matrix.dictionary()) {
            const auto& [row, col] = index_pair;
            m_offsets_map.emplace(index_pair, m_values.size());
            m_values.push_back(value);
            m_row_indices.push_back(row);
            m_col_indices.push_back(col);
        }
    }

    [[using gnu: ]] operator DokMatrix<value_type, index_type, allocator_type>() const {
        DokMatrix<value_type, index_type, allocator_type> dok_matrix(
            m_nrows, m_ncols, m_values.get_allocator()
        );
        const size_type nnzs{m_values.size()};
        dok_matrix.reserve(nnzs);
        for (size_type i{0}; i < nnzs; ++i) {
            dok_matrix.updateCoeff(m_row_indices[i], m_col_indices[i], m_values[i]);
        }
        return dok_matrix;
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
        return m_values.size();
    }

    [[using gnu: ]]
    auto resize(size_type nrows, size_type ncols) -> void {
        if (nrows < m_nrows || ncols < m_ncols) {
            const size_type nnzs{m_values.size()};
            size_type j{0};
            for (size_type i{0}; i < nnzs; ++i) {
                if (m_row_indices[i] >= static_cast<index_type>(nrows) ||
                    m_col_indices[i] >= static_cast<index_type>(ncols)) {
                    m_offsets_map.erase({m_row_indices[i], m_col_indices[i]});
                    continue;
                }
                m_offsets_map[{m_row_indices[i], m_col_indices[i]}] = j;
                m_values[j] = m_values[i];
                m_row_indices[j] = m_row_indices[i];
                m_col_indices[j] = m_col_indices[i];
                ++j;
            }
            m_values.resize(j);
            m_row_indices.resize(j);
            m_col_indices.resize(j);
            m_values.shrink_to_fit();
            m_row_indices.shrink_to_fit();
            m_col_indices.shrink_to_fit();
        }
        m_nrows = nrows;
        m_ncols = ncols;
        return;
    }

    [[using gnu: always_inline]]
    auto reserve(size_type capacity) -> void {
        m_offsets_map.reserve(capacity);
        m_values.reserve(capacity);
        m_row_indices.reserve(capacity);
        m_col_indices.reserve(capacity);
        return;
    }

    [[using gnu: always_inline]]
    auto clear() -> void {
        m_offsets_map.clear();
        m_values.clear();
        m_row_indices.clear();
        m_col_indices.clear();
        return;
    }

    [[using gnu: ]]
    auto compress() -> void {
#ifndef BASJOO_USE_BOOST_UNORDERED
        using std::erase_if;
#else
        using boost::unordered::erase_if;
#endif
        const auto count =
            erase_if(m_offsets_map, [this](const auto& item) constexpr noexcept -> bool {
                return m_values[item.second] == static_cast<value_type>(0.0);
            });
        if (count == 0) {
            return;
        }
        size_type j{0};
        for (auto& [index_pair, offset] : m_offsets_map) {
            const auto& [row, col] = index_pair;
            m_values[j] = m_values[offset];
            m_row_indices[j] = row;
            m_col_indices[j] = col;
            offset = j;
            ++j;
        }
        m_values.resize(j);
        m_row_indices.resize(j);
        m_col_indices.resize(j);
        m_values.shrink_to_fit();
        m_row_indices.shrink_to_fit();
        m_col_indices.shrink_to_fit();
        return;
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto coeff(size_type row, size_type col) const noexcept -> value_type {
        if (row >= m_nrows || col >= m_ncols) [[unlikely]] {
            return static_cast<value_type>(0.0);
        }
        if (const auto search =
                m_offsets_map.find({static_cast<index_type>(row), static_cast<index_type>(col)});
            search != m_offsets_map.cend()) {
            return m_values[search->second];
        }
        return static_cast<value_type>(0.0);
    }

    [[using gnu: flatten, leaf, hot]]
    auto updateCoeff(size_type row, size_type col, const_reference value) -> void {
        if (row >= m_nrows || col >= m_ncols) [[unlikely]] {
            return;
        }
        if (const auto search =
                m_offsets_map.find({static_cast<index_type>(row), static_cast<index_type>(col)});
            search != m_offsets_map.cend()) {
            m_values[search->second] = value;
        } else {
            m_offsets_map.emplace(
                decltype(search->first){static_cast<index_type>(row), static_cast<index_type>(col)},
                m_values.size()
            );
            m_values.push_back(value);
            m_row_indices.push_back(row);
            m_col_indices.push_back(col);
        }
        return;
    }

    [[using gnu: flatten, leaf, hot]]
    auto eraseCoeff(size_type row, size_type col) -> void {
        if (row >= m_nrows || col >= m_ncols) [[unlikely]] {
            return;
        }
        if (const auto search =
                m_offsets_map.find({static_cast<index_type>(row), static_cast<index_type>(col)});
            search != m_offsets_map.cend()) {
            const size_type offset{search->second};
            for (auto& item : m_offsets_map) {
                if (item.second > offset) {
                    --item.second;
                }
            }
            m_offsets_map.erase(search);
            m_values.erase(m_values.begin() + offset);
            m_row_indices.erase(m_row_indices.begin() + offset);
            m_col_indices.erase(m_col_indices.begin() + offset);
        }
        return;
    }

    [[using gnu: pure, always_inline, hot]]
    auto operator[](size_type row, size_type col) const noexcept -> value_type {
        return coeff(row, col);
    }

    [[using gnu: pure, always_inline, leaf]]
    auto values() const noexcept -> std::span<const value_type> {
        return m_values;
    }

    [[using gnu: pure, always_inline, leaf]]
    auto rowIndices() const noexcept -> std::span<const index_type> {
        return m_row_indices;
    }

    [[using gnu: pure, always_inline, leaf]]
    auto colIndices() const noexcept -> std::span<const index_type> {
        return m_col_indices;
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<6>;

    size_type m_nrows;
    size_type m_ncols;
    dictionary_type m_offsets_map;
    std::vector<value_type, allocator_type> m_values;
    std::vector<
        index_type,
        typename std::allocator_traits<allocator_type>::template rebind_alloc<index_type>>
        m_row_indices;
    std::vector<
        index_type,
        typename std::allocator_traits<allocator_type>::template rebind_alloc<index_type>>
        m_col_indices;
};

} // namespace basjoo::math
