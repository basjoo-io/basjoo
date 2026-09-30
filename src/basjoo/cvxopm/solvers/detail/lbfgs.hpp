/**
 * @file lbfgs.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-11-16
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <deque>
#include <format>
#include <memory_resource>

#include "basjoo/math/concepts.hpp"

namespace basjoo::cvxopm::detail {

template <::basjoo::math::VecArithmetic T>
class LbfgsHessinv final {
  public:
    struct BfgsNode final {
        T p;
        T dg;
        typename T::value_type rho;
        typename T::value_type alpha;
    };

    using param_type = T;
    using scalar_type = typename param_type::value_type;
    using size_type = typename param_type::size_type;
    using value_type = BfgsNode;
    using allocator_type = std::pmr::polymorphic_allocator<value_type>;

    LbfgsHessinv() noexcept = default;
    LbfgsHessinv(const LbfgsHessinv& other) noexcept = delete;
    auto operator=(const LbfgsHessinv& other) noexcept = delete;
    LbfgsHessinv(LbfgsHessinv&& other) noexcept = delete;
    auto operator=(LbfgsHessinv&& other) noexcept = delete;
    ~LbfgsHessinv() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_bfgs_nodes.get_allocator();
    }

    [[using gnu: always_inline]]
    LbfgsHessinv(size_type n, size_type m, const allocator_type& alloc = {}) noexcept
        : m_num_dims{n}, m_num_nodes{m}, m_bfgs_nodes(alloc) {}

    [[using gnu: pure, always_inline]]
    auto nrows() const noexcept -> size_type {
        return m_num_dims;
    }

    [[using gnu: pure, always_inline]]
    auto ncols() const noexcept -> size_type {
        return m_num_dims;
    }

    [[using gnu: pure, always_inline]]
    auto dot(param_type x) noexcept(!BASJOO_CHECK_PARAMS) -> param_type {
#if BASJOO_CHECK_PARAMS == 1
        if (ncols() != x.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::cvxopm::detail::LbfgsHessinv::dot() "
                    "requires the ncols() is identical to the size of the array: this->ncols() == "
                    "{0:d} while x.size() == {1:d}.",
                    ncols(), x.size()
                )
            );
        }
#endif
        if (m_bfgs_nodes.empty()) {
            return x;
        }

        for (auto it = m_bfgs_nodes.rbegin(); it != m_bfgs_nodes.rend(); ++it) {
            it->alpha = x.dot(it->p) * it->rho;
            x -= it->dg * it->alpha;
        }
        // x /= m_bfgs_nodes.back().rho * m_bfgs_nodes.back().dg.dot(m_bfgs_nodes.back().dg);
        for (auto it = m_bfgs_nodes.cbegin(); it != m_bfgs_nodes.cend(); ++it) {
            x += it->p * (it->alpha - x.dot(it->dg) * it->rho);
        }

        return x;
    }

    [[using gnu: flatten, leaf, hot]]
    auto update(param_type p, param_type dg) noexcept(!BASJOO_CHECK_PARAMS) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (!(ncols() == p.size() && (ncols() == dg.size()))) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::cvxopm::detail::LbfgsHessinv::update() "
                    "requires the ncols() is identical to the size of the two array: this->ncols() "
                    "== {0:d} while p.size() == {1:d}, dg.size() == {2:d}.",
                    ncols(), p.size(), dg.size()
                )
            );
        }
#endif
        if (m_bfgs_nodes.size() == m_num_nodes) {
            m_bfgs_nodes.pop_front();
        }
        const scalar_type pdg = p.dot(dg);
        m_bfgs_nodes.emplace_back(
            BfgsNode{
                std::move(p), std::move(dg), static_cast<scalar_type>(1.0) / pdg,
                static_cast<scalar_type>(0.0)
            }
        );

        return;
    }

  private:
    size_type m_num_dims;
    size_type m_num_nodes;
    std::deque<BfgsNode, allocator_type> m_bfgs_nodes;
};

} // namespace basjoo::cvxopm::detail
