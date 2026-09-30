/**
 * @file bfgs.hpp
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
#include <cmath>
#include <format>
#include <limits>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/detail/dense_generate_trait.hpp"

namespace basjoo::cvxopm::detail {

template <::basjoo::math::VecArithmetic T>
class BfgsHessinv final {
  public:
    using param_type = T;
    using value_type = typename param_type::value_type;
    using size_type = typename param_type::size_type;
    using allocator_type = typename param_type::allocator_type;

    BfgsHessinv() noexcept = default;
    BfgsHessinv(const BfgsHessinv& other) noexcept = delete;
    auto operator=(const BfgsHessinv& other) noexcept = delete;
    BfgsHessinv(BfgsHessinv&& other) noexcept = delete;
    auto operator=(BfgsHessinv&& other) noexcept = delete;
    ~BfgsHessinv() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_data.get_allocator();
    }

    [[using gnu: always_inline]]
    BfgsHessinv(size_type n, const allocator_type& alloc = {}) noexcept
        : m_data(n, n, alloc) {
        m_data.setIdentity();
    }

    [[using gnu: pure, always_inline]]
    auto nrows() const noexcept -> size_type {
        return m_data.nrows();
    }

    [[using gnu: pure, always_inline]]
    auto ncols() const noexcept -> size_type {
        return m_data.ncols();
    }

    [[using gnu: pure, always_inline]]
    auto dot(const param_type& x) const noexcept -> param_type {
        return m_data.dot(x);
    }

    [[using gnu: flatten, leaf, hot]]
    auto update(const param_type& p, param_type dg) noexcept(!BASJOO_CHECK_PARAMS) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (!(ncols() == p.size() && (ncols() == dg.size()))) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::cvxopm::detail::BfgsHessinv::update() "
                    "requires the ncols() is identical to the size of the two array: this->ncols() "
                    "== {0:d} while p.size() == {1:d}, dg.size() == {2:d}.",
                    ncols(), p.size(), dg.size()
                )
            );
        }
#endif
        const size_type n{m_data.nrows()};
        param_type hdg = m_data.dot(dg);
        value_type fac = p.dot(dg);
        const value_type fae = dg.dot(hdg);
        const value_type sqr_dg = dg.dot(dg);
        const value_type sqr_p = p.dot(p);

        if (fac > std::sqrt(std::numeric_limits<value_type>::epsilon() * sqr_dg * sqr_p)) {
            fac = static_cast<value_type>(1.0) / fac;
            const value_type fad = static_cast<value_type>(1.0) / fae;
            std::transform(
                p.data(), p.data() + p.size(), hdg.data(), dg.data(),
                [fac, fad](const value_type& a, const value_type& b) noexcept -> value_type {
                    return fac * a - fad * b;
                }
            );
            for (size_type j{0}; j < n; ++j) {
                for (size_type i{j}; i < n; ++i) {
                    m_data[i, j] += fac * p[i] * p[j] - fad * hdg[i] * hdg[j] + fae * dg[i] * dg[j];
                    m_data[j, i] = m_data[i, j];
                }
            }
        }

        return;
    }

  private:
    ::basjoo::math::detail::DenseGenerateTraitT<param_type> m_data;
};

} // namespace basjoo::cvxopm::detail
