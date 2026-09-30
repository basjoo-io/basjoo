/**
 * @file qdldl_dcmp.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-06-04
 *
 * @copyright Copyright (c) 2026 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <stdexcept>
#include <vector>

#include "qdldl.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/sparse/csc_matrix.hpp"

namespace basjoo::math {

template <SparseMatArithmetic T>
    requires std::same_as<typename T::value_type, QDLDL_float> &&
             std::same_as<typename T::index_type, QDLDL_int>
class QdldlDcmp final {
  public:
    using matrix_type = std::remove_cvref_t<T>;
    using value_type = typename matrix_type::value_type;
    using index_type = typename matrix_type::index_type;
    using size_type = typename matrix_type::size_type;
    using allocator_type = typename matrix_type::allocator_type;

    QdldlDcmp() noexcept = delete;
    QdldlDcmp(const QdldlDcmp& other) noexcept = delete;
    auto operator=(const QdldlDcmp& other) noexcept -> QdldlDcmp& = delete;
    QdldlDcmp(QdldlDcmp&& other) noexcept = delete;
    auto operator=(QdldlDcmp&& other) noexcept -> QdldlDcmp& = delete;
    ~QdldlDcmp() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_dinv.get_allocator();
    }

    explicit QdldlDcmp(const matrix_type& matrix)
        : m_dinv(matrix.nrows(), matrix.get_allocator()), m_lx(0, matrix.get_allocator()),
          m_lp(matrix.nrows() + 1, matrix.get_allocator()), m_li(0, matrix.get_allocator()) {
#if BASJOO_CHECK_PARAMS == 1
        if (matrix.nrows() == 0) [[unlikely]] {
            throw std::invalid_argument("QdldlDcmp: matrix must be non-empty.");
        }
        if (matrix.nrows() != matrix.ncols()) [[unlikely]] {
            throw std::invalid_argument("QdldlDcmp: matrix must be square.");
        }
#endif
        /*--------------------------------
         * pre-factorisation memory allocations
         *---------------------------------*/
        const index_type num_dims = m_dinv.size();
        const CscMatrix<value_type, index_type, allocator_type>& csc_matrix = matrix;
        std::vector<
            index_type,
            typename std::allocator_traits<allocator_type>::template rebind_alloc<index_type>>
            etree(num_dims, get_allocator());
        std::vector<
            index_type,
            typename std::allocator_traits<allocator_type>::template rebind_alloc<index_type>>
            lnz(num_dims, get_allocator());
        std::vector<value_type, allocator_type> d(num_dims, get_allocator());
        std::vector<
            index_type,
            typename std::allocator_traits<allocator_type>::template rebind_alloc<index_type>>
            iwork(3 * num_dims, get_allocator());
        std::vector<
            std::uint8_t,
            typename std::allocator_traits<allocator_type>::template rebind_alloc<std::uint8_t>>
            bwork(num_dims, get_allocator());
        std::vector<value_type, allocator_type> fwork(num_dims, get_allocator());

        /*--------------------------------
         * elimination tree calculation
         *---------------------------------*/
        const index_type sum_lnz = QDLDL_etree(
            num_dims, csc_matrix.outerIndices().data(), csc_matrix.innerIndices().data(),
            iwork.data(), lnz.data(), etree.data()
        );
        if (sum_lnz < 0) {
            throw std::invalid_argument(
                "QdldlDcmp: matrix is not upper triangular or has an empty column."
            );
        }
        m_lx.resize(sum_lnz);
        m_li.resize(sum_lnz);

        /*--------------------------------
         * LDL factorisation
         *---------------------------------*/
        const index_type num_pos = QDLDL_factor(
            num_dims, csc_matrix.outerIndices().data(), csc_matrix.innerIndices().data(),
            csc_matrix.values().data(), m_lp.data(), m_li.data(), m_lx.data(), d.data(),
            m_dinv.data(), lnz.data(), etree.data(), bwork.data(), iwork.data(), fwork.data()
        );
        if (num_pos < 0) {
            throw std::runtime_error(
                "QdldlDcmp: LDL^T factorization failed — matrix has a zero diagonal element."
            );
        }

        return;
    }

    template <typename VectorType>
        requires requires(std::remove_cvref_t<VectorType> b) {
            { b.data() } -> std::same_as<value_type*>;
        }
    [[nodiscard]]
    auto solve(VectorType b) const noexcept(!BASJOO_CHECK_PARAMS) -> VectorType {
#if BASJOO_CHECK_PARAMS == 1
        if (b.size() != m_dinv.size()) [[unlikely]] {
            throw std::invalid_argument(
                "QdldlDcmp::solve: vector size does not match matrix dimension."
            );
        }
#endif
        QDLDL_solve(m_dinv.size(), m_lp.data(), m_li.data(), m_lx.data(), m_dinv.data(), b.data());
        return b;
    }

  private:
    std::vector<value_type, allocator_type> m_dinv, m_lx;
    std::vector<
        index_type,
        typename std::allocator_traits<allocator_type>::template rebind_alloc<index_type>>
        m_lp, m_li;
};

} // namespace basjoo::math
