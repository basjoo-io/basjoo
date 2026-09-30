/**
 * @file lil_matrix.hpp
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

#include <algorithm>
#include <concepts>
#include <initializer_list>
#include <ranges>
#include <utility>

#include "zpp_bits.h"

#include "basjoo/common/aligned_allocator.hpp"
#include "basjoo/math/concepts.hpp"
#include "basjoo/math/sparse/sparse_traits.hpp"

namespace basjoo::math {

template <ScalarArithmetic Scalar, std::integral Index, Allocatory Alloc>
class LilMatrix final {
  public:
    using value_type = typename SparseTraits<LilMatrix>::value_type;
    using index_type = typename SparseTraits<LilMatrix>::index_type;
    using reference = typename SparseTraits<LilMatrix>::reference;
    using const_reference = typename SparseTraits<LilMatrix>::const_reference;
    using size_type = typename SparseTraits<LilMatrix>::size_type;
    using difference_type = typename SparseTraits<LilMatrix>::difference_type;
    using allocator_type = typename SparseTraits<LilMatrix>::allocator_type;
    using dictionary_type = typename SparseTraits<LilMatrix>::dictionary_type;

    LilMatrix() noexcept = default;
    LilMatrix(const LilMatrix& other) = default;
    auto operator=(const LilMatrix& other) -> LilMatrix& = default;
    LilMatrix(LilMatrix&& other) noexcept = default;
    auto operator=(LilMatrix&& other) noexcept -> LilMatrix& = default;
    ~LilMatrix() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_row_dictionaries.get_allocator();
    }

    [[using gnu: always_inline]]
    explicit LilMatrix(const allocator_type& alloc) noexcept
        : m_nrows{0}, m_ncols{0}, m_nnzs{0}, m_row_dictionaries(alloc) {}

    [[using gnu: always_inline]]
    explicit LilMatrix(size_type nrows, size_type ncols, const allocator_type& alloc = {}) noexcept
        : m_nrows{nrows}, m_ncols{ncols}, m_nnzs{0}, m_row_dictionaries(alloc) {}

    [[using gnu: always_inline]]
    LilMatrix(const DokMatrix<value_type, index_type, allocator_type>& dok_matrix)
        : m_nrows{dok_matrix.nrows()}, m_ncols{dok_matrix.ncols()}, m_nnzs{dok_matrix.nnzs()},
          m_row_dictionaries(dok_matrix.get_allocator()) {
        for (const auto& [index_pair, value] : dok_matrix.dictionary()) {
            m_row_dictionaries[index_pair.row].emplace(index_pair.col, value);
        }
    }

    [[using gnu: ]] operator DokMatrix<value_type, index_type, allocator_type>() const {
        DokMatrix<value_type, index_type, allocator_type> dok_matrix(
            m_nrows, m_ncols, get_allocator()
        );
        for (const auto& [row, row_dictionary] : m_row_dictionaries) {
            for (const auto& [col, value] : row_dictionary) {
                dok_matrix.updateCoeff(row, col, value);
            }
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

    [[using gnu: pure, always_inline, leaf]]
    auto nnzs() const noexcept -> size_type {
        return m_nnzs;
    }

    [[using gnu: ]]
    auto resize(size_type nrows, size_type ncols) -> void {
#ifndef BASJOO_USE_BOOST_UNORDERED
        using std::erase_if;
#else
        using boost::unordered::erase_if;
#endif
        if (m_nrows > nrows) {
            erase_if(
                m_row_dictionaries, [nrows](const auto& row_dictionary) constexpr noexcept -> bool {
                    return row_dictionary.first >= static_cast<index_type>(nrows);
                }
            );
        }
        if (m_ncols > ncols) {
            for (auto& [row, row_dictionary] : m_row_dictionaries) {
                erase_if(row_dictionary, [ncols](const auto& item) constexpr noexcept -> bool {
                    return item.first >= static_cast<index_type>(ncols);
                });
            }
            erase_if(m_row_dictionaries, [](const auto& row_dictionary) constexpr noexcept -> bool {
                return row_dictionary.second.empty();
            });
        }
        if (m_nrows > nrows || m_ncols > ncols) {
            m_nnzs = 0;
            for (const auto& [row, row_dictionary] : m_row_dictionaries) {
                m_nnzs += row_dictionary.size();
            }
        }
        m_nrows = nrows;
        m_ncols = ncols;
        return;
    }

    [[using gnu: always_inline]]
    auto reserve(size_type capacity) -> void {
        m_row_dictionaries.reserve(capacity);
        return;
    }

    [[using gnu: always_inline]]
    auto clear() -> void {
        m_nnzs = 0;
        m_row_dictionaries.clear();
        return;
    }

    [[using gnu: always_inline]]
    auto compress() -> void {
#ifndef BASJOO_USE_BOOST_UNORDERED
        using std::erase_if;
#else
        using boost::unordered::erase_if;
#endif
        for (auto& [row, row_dictionary] : m_row_dictionaries) {
            erase_if(row_dictionary, [](const auto& item) constexpr noexcept -> bool {
                return item.second == value_type(0.0);
            });
        }
        erase_if(m_row_dictionaries, [](const auto& row_dictionary) constexpr noexcept -> bool {
            return row_dictionary.second.empty();
        });
        m_nnzs = 0;
        for (const auto& [row, row_dictionary] : m_row_dictionaries) {
            m_nnzs += row_dictionary.size();
        }
        return;
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto coeff(size_type row, size_type col) const noexcept -> value_type {
        if (row >= m_nrows || col >= m_ncols) [[unlikely]] {
            return static_cast<value_type>(0.0);
        }
        if (const auto search_row = m_row_dictionaries.find(static_cast<index_type>(row));
            search_row != m_row_dictionaries.cend()) {
            if (const auto search_col = search_row->second.find(static_cast<index_type>(col));
                search_col != search_row->second.cend()) {
                return search_col->second;
            }
        }
        return static_cast<value_type>(0.0);
    }

    [[using gnu: flatten, leaf, hot]]
    auto updateCoeff(size_type row, size_type col, const_reference value) -> void {
        if (row >= m_nrows || col >= m_ncols) [[unlikely]] {
            return;
        }
        if (auto search_row = m_row_dictionaries.find(static_cast<index_type>(row));
            search_row != m_row_dictionaries.end()) {
            if (auto search_col = search_row->second.find(static_cast<index_type>(col));
                search_col != search_row->second.end()) {
                search_col->second = value;
            } else {
                m_nnzs += 1;
                search_row->second.emplace(static_cast<index_type>(col), value);
            }
        } else {
            if (value != value_type(0.0)) {
                m_nnzs += 1;
                m_row_dictionaries.emplace(
                    static_cast<index_type>(row),
                    decltype(search_row->second)(
                        {{static_cast<index_type>(col), value}}, get_allocator()
                    )
                );
            }
        }
        return;
    }

    [[using gnu: flatten, leaf, hot]]
    auto eraseCoeff(size_type row, size_type col) -> void {
        if (row >= m_nrows || col >= m_ncols) [[unlikely]] {
            return;
        }
        if (auto search_row = m_row_dictionaries.find(static_cast<index_type>(row));
            search_row != m_row_dictionaries.end()) {
            if (auto search_col = search_row->second.find(static_cast<index_type>(col));
                search_col != search_row->second.end()) {
                m_nnzs -= 1;
                if (search_row->second.size() != 1) {
                    search_row->second.erase(search_col);
                } else {
                    m_row_dictionaries.erase(search_row);
                }
            }
        }
        return;
    }

    [[using gnu: pure, always_inline, hot]]
    auto operator[](size_type row, size_type col) const noexcept -> value_type {
        return coeff(row, col);
    }

    template <
        std::ranges::input_range R = std::initializer_list<std::pair<const index_type, value_type>>>
    [[using gnu: flatten, leaf, hot]]
    auto updateRow(size_type row, R&& row_vector) -> void
        requires std::same_as<
            std::ranges::range_value_t<R>, std::pair<const index_type, value_type>>
    {
#ifndef BASJOO_USE_BOOST_UNORDERED
        using std::erase_if;
#else
        using boost::unordered::erase_if;
#endif
        if (row >= m_nrows) [[unlikely]] {
            return;
        }
        if (auto search = m_row_dictionaries.find(static_cast<index_type>(row));
            search != m_row_dictionaries.end()) {
            m_nnzs -= search->second.size();
            m_row_dictionaries.erase(search);
        }
        const size_type row_capacity{[&row_vector]() constexpr noexcept -> size_type {
            if constexpr (std::ranges::sized_range<R>) {
                return static_cast<size_type>(std::ranges::size(row_vector));
            } else {
                return 0;
            }
        }()};
        typename dictionary_type::mapped_type row_dictionary(
            row_vector.begin(), row_vector.end(), row_capacity, get_allocator()
        );
        erase_if(
            row_dictionary,
            [ncols = static_cast<index_type>(m_ncols)](const auto& item) constexpr noexcept
                -> bool { return item.first >= ncols; }
        );
        m_nnzs += row_dictionary.size();
        m_row_dictionaries.emplace(static_cast<index_type>(row), std::move(row_dictionary));
        return;
    }

    [[using gnu: flatten, leaf, hot]]
    auto eraseRow(size_type row) -> void {
        if (row >= m_nrows) [[unlikely]] {
            return;
        }
        if (auto search = m_row_dictionaries.find(static_cast<index_type>(row));
            search != m_row_dictionaries.end()) {
            m_nnzs -= search->second.size();
            m_row_dictionaries.erase(search);
        }
    }

    [[using gnu: always_inline, leaf]]
    auto row_dictionaries() const noexcept -> const dictionary_type& {
        return m_row_dictionaries;
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<4>;

    size_type m_nrows;
    size_type m_ncols;
    size_type m_nnzs;
    dictionary_type m_row_dictionaries;
};

} // namespace basjoo::math
