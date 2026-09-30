/**
 * @file route_line_cubic_acc_planner.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-15
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <format>
#include <limits>
#include <memory_resource>
#include <ranges>
#include <span>
#include <stdexcept>
#include <utility>
#include <vector>

#include "basjoo/bicycle/planners/fence.hpp"
#include "basjoo/bicycle/planners/motion.hpp"
#include "basjoo/common/logging.hpp"
#include "basjoo/cvxopm/info.hpp"
#include "basjoo/cvxopm/problems/osqp_problem.hpp"
#include "basjoo/cvxopm/settings.hpp"
#include "basjoo/cvxopm/solvers/osqp_solver.hpp"
#include "basjoo/math/functions/piecewise_linear_function.hpp"

namespace basjoo::bicycle {

template <std::floating_point T>
class RouteLineCubicAccPlanner final {
  public:
    using value_type = T;
    using scalar_type = T;
    using index_type = int;
    using size_type = std::size_t;
    using allocator_type = std::pmr::polymorphic_allocator<value_type>;

    RouteLineCubicAccPlanner(const RouteLineCubicAccPlanner& other) noexcept = delete;
    auto operator=(const RouteLineCubicAccPlanner& other) noexcept
        -> RouteLineCubicAccPlanner& = delete;
    RouteLineCubicAccPlanner(RouteLineCubicAccPlanner&& other) noexcept = delete;
    auto operator=(RouteLineCubicAccPlanner&& other) noexcept -> RouteLineCubicAccPlanner& = delete;
    ~RouteLineCubicAccPlanner() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_settings.memory_resource;
    }

    template <std::ranges::input_range R = std::initializer_list<value_type>>
    explicit RouteLineCubicAccPlanner(
        R&& sample_ts, const ::basjoo::cvxopm::Settings<scalar_type, index_type>& settings = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        requires std::same_as<std::ranges::range_value_t<R>, value_type>
        : m_num_samples(sample_ts.size()), m_time_scale(sample_ts.back() - sample_ts.front()),
          m_osqp_problem(sample_ts.size() * 2, sample_ts.size() * 4, settings.memory_resource),
          m_settings{settings},
          m_sample_ts(sample_ts.cbegin(), sample_ts.cend(), settings.memory_resource),
          m_hs(sample_ts.size() - 1, settings.memory_resource),
          m_h2s(sample_ts.size() - 1, settings.memory_resource),
          m_reciprocal_hs(sample_ts.size() - 1, settings.memory_resource),
          m_reciprocal_h2s(sample_ts.size() - 1, settings.memory_resource) {
#if BASJOO_CHECK_PARAMS == 1
        if (sample_ts.size() < 2) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid argument error detected: size of sample_ts must larger than 2: "
                    "sample_ts.size() = {0:d}.",
                    sample_ts.size()
                )
            );
        }
#endif
        for (size_type i{0}; i < m_sample_ts.size() - 1; ++i) {
            const scalar_type diff = m_sample_ts[i + 1] - m_sample_ts[i];
            const scalar_type reciprocal_diff = 1.0 / diff;
            m_hs[i] = diff;
            m_h2s[i] = diff * diff;
            m_reciprocal_hs[i] = reciprocal_diff;
            m_reciprocal_h2s[i] = reciprocal_diff * reciprocal_diff;
        }
        setIntegrationRelation();
    }

    [[using gnu: pure, always_inline]]
    auto num_samples() const noexcept -> size_type {
        return m_num_samples;
    }

    [[using gnu: pure, always_inline]]
    auto osqp_problem() const noexcept
        -> const ::basjoo::cvxopm::OsqpProblem<scalar_type, index_type>& {
        return m_osqp_problem;
    }

    [[using gnu: pure, always_inline]]
    auto settings() const noexcept -> const ::basjoo::cvxopm::Settings<scalar_type, index_type>& {
        return m_settings;
    }

    auto setHardFences(std::span<const HardFence<scalar_type>> hard_fences) noexcept -> void {
        std::pmr::vector<scalar_type> lower_bound(
            m_num_samples, std::numeric_limits<scalar_type>::lowest(), get_allocator()
        );
        std::pmr::vector<scalar_type> upper_bound(
            m_num_samples, std::numeric_limits<scalar_type>::max(), get_allocator()
        );
        for (const auto& hard_fence : hard_fences) {
            const index_type istart =
                ::basjoo::math::nearestUpperElement(m_sample_ts, hard_fence.bound_ts.front()) -
                m_sample_ts.cbegin();
            const index_type iend =
                ::basjoo::math::nearestUpperElement(m_sample_ts, hard_fence.bound_ts.back()) -
                m_sample_ts.cbegin();
            if (istart == m_num_samples) {
                basjoo::common::logWarn(
                    "Invalid argument issue detected! The front() of hard_fence.bound_ts should be "
                    "less than m_sample_ts.back(): hard_fence.bound_ts.front() = {0:f} while "
                    "m_sample_ts.back() = {1:f}.",
                    hard_fence.bound_ts.front(), m_sample_ts.back()
                );
                continue;
            }
            if (iend == 0) {
                basjoo::common::logWarn(
                    "Invalid argument issue detected! The back() of hard_fence.bound_ts should be "
                    "larger than m_sample_ts.front(): hard_fence.bound_ts.back() = {0:f} while "
                    "m_sample_ts.front() = {1:f}.",
                    hard_fence.bound_ts.back(), m_sample_ts.front()
                );
                continue;
            }

            const ::basjoo::math::pmr::PiecewiseLinearFunction<value_type> bound_func{
                hard_fence.bound_ts, hard_fence.bound_ss, get_allocator()
            };

            if (hard_fence.actio == ::basjoo::bicycle::Actio::PUSHING) {
                if (istart != 0) {
                    const scalar_type h{m_sample_ts[istart] - m_sample_ts[istart - 1]};
                    const scalar_type ratio{
                        (hard_fence.bound_ts.front() - m_sample_ts[istart - 1]) / h
                    };
                    const std::array<scalar_type, 4> proration_coeffs = prorationCoeffs(ratio, h);
                    m_osqp_problem.addConstraintTerm(
                        {{sIndex(istart - 1), proration_coeffs[0]},
                         {sIndex(istart), proration_coeffs[1]},
                         {vIndex(istart - 1), proration_coeffs[2]},
                         {vIndex(istart), proration_coeffs[3]}},
                        hard_fence.bound_ss.front(), std::numeric_limits<scalar_type>::max()
                    );
                }
                for (index_type i{istart}; i < iend; ++i) {
                    lower_bound[i] = std::max(bound_func(m_sample_ts[i]), lower_bound[i]);
                }
                if (iend != m_num_samples) {
                    const scalar_type h{m_sample_ts[iend] - m_sample_ts[iend - 1]};
                    const scalar_type ratio{
                        (hard_fence.bound_ts.back() - m_sample_ts[iend - 1]) / h
                    };
                    const std::array<scalar_type, 4> proration_coeffs = prorationCoeffs(ratio, h);
                    m_osqp_problem.addConstraintTerm(
                        {{sIndex(iend - 1), proration_coeffs[0]},
                         {sIndex(iend), proration_coeffs[1]},
                         {vIndex(iend - 1), proration_coeffs[2]},
                         {vIndex(iend), proration_coeffs[3]}},
                        hard_fence.bound_ss.back(), std::numeric_limits<scalar_type>::max()
                    );
                }
            } else {
                if (istart != 0) {
                    const scalar_type h{m_sample_ts[istart] - m_sample_ts[istart - 1]};
                    const scalar_type ratio{
                        (hard_fence.bound_ts.front() - m_sample_ts[istart - 1]) / h
                    };
                    const std::array<scalar_type, 4> proration_coeffs = prorationCoeffs(ratio, h);
                    m_osqp_problem.addConstraintTerm(
                        {{sIndex(istart - 1), proration_coeffs[0]},
                         {sIndex(istart), proration_coeffs[1]},
                         {vIndex(istart - 1), proration_coeffs[2]},
                         {vIndex(istart), proration_coeffs[3]}},
                        std::numeric_limits<scalar_type>::lowest(), hard_fence.bound_ss.front()
                    );
                }
                for (index_type i{istart}; i < iend; ++i) {
                    upper_bound[i] = std::min(bound_func(m_sample_ts[i]), upper_bound[i]);
                }
                if (iend != m_num_samples) {
                    const scalar_type h{m_sample_ts[iend] - m_sample_ts[iend - 1]};
                    const scalar_type ratio{
                        (hard_fence.bound_ts.back() - m_sample_ts[iend - 1]) / h
                    };
                    const std::array<scalar_type, 4> proration_coeffs = prorationCoeffs(ratio, h);
                    m_osqp_problem.addConstraintTerm(
                        {{sIndex(iend - 1), proration_coeffs[0]},
                         {sIndex(iend), proration_coeffs[1]},
                         {vIndex(iend - 1), proration_coeffs[2]},
                         {vIndex(iend), proration_coeffs[3]}},
                        std::numeric_limits<scalar_type>::lowest(), hard_fence.bound_ss.back()
                    );
                }
            }
        }

        for (index_type i{1}; i < m_num_samples - 1; ++i) {
            m_osqp_problem.updateConstraintTerm(
                sIndex(i), {{sIndex(i), 1.0}}, lower_bound[i], upper_bound[i]
            );
        }
        return;
    }

    auto setSoftFences(std::span<const SoftFence<scalar_type>> soft_fences) noexcept -> void {
        for (const auto& soft_fence : soft_fences) {
            const index_type istart =
                ::basjoo::math::nearestUpperElement(m_sample_ts, soft_fence.bound_ts.front()) -
                m_sample_ts.cbegin();
            const index_type iend =
                ::basjoo::math::nearestUpperElement(m_sample_ts, soft_fence.bound_ts.back()) -
                m_sample_ts.cbegin();
            if (istart == m_num_samples) {
                basjoo::common::logWarn(
                    "Invalid argument issue detected! The front() of hard_fence.bound_ts should be "
                    "less than m_sample_ts.back(): hard_fence.bound_ts.front() = {0:f} while "
                    "m_sample_ts.back() = {1:f}.",
                    soft_fence.bound_ts.front(), m_sample_ts.back()
                );
                continue;
            }
            if (iend == 0) {
                basjoo::common::logWarn(
                    "Invalid argument issue detected! The back() of hard_fence.bound_ts should be "
                    "larger than m_sample_ts.front(): hard_fence.bound_ts.back() = {0:f} while "
                    "m_sample_ts.front() = {1:f}.",
                    soft_fence.bound_ts.back(), m_sample_ts.front()
                );
                continue;
            }

            const ::basjoo::math::pmr::PiecewiseLinearFunction<value_type> bound_func{
                soft_fence.bound_ts, soft_fence.bound_ss, get_allocator()
            };

            scalar_type factor{0.0};

            if (soft_fence.actio == ::basjoo::bicycle::Actio::PUSHING) {
                if (istart == iend) {
                    factor = (soft_fence.bound_ts.back() - soft_fence.bound_ts.front()) /
                             m_time_scale * 0.5;
                    const scalar_type h{m_sample_ts[istart] - m_sample_ts[istart - 1]};
                    scalar_type ratio{(soft_fence.bound_ts.front() - m_sample_ts[istart - 1]) / h};
                    std::array<scalar_type, 4> proration_coeffs = prorationCoeffs(ratio, h);
                    m_osqp_problem.addRampCostTerm(
                        {{sIndex(istart - 1), -proration_coeffs[0]},
                         {sIndex(istart), -proration_coeffs[1]},
                         {vIndex(istart - 1), -proration_coeffs[2]},
                         {vIndex(istart), -proration_coeffs[3]}},
                        -soft_fence.bound_ss.front(), soft_fence.linear_weight * factor,
                        soft_fence.quadratic_weight * factor
                    );
                    ratio = (soft_fence.bound_ts.back() - m_sample_ts[iend - 1]) / h;
                    proration_coeffs = prorationCoeffs(ratio, h);
                    m_osqp_problem.addRampCostTerm(
                        {{sIndex(iend - 1), -proration_coeffs[0]},
                         {sIndex(iend), -proration_coeffs[1]},
                         {vIndex(iend - 1), -proration_coeffs[2]},
                         {vIndex(iend), -proration_coeffs[3]}},
                        -soft_fence.bound_ss.back(), soft_fence.linear_weight * factor,
                        soft_fence.quadratic_weight * factor
                    );
                    continue;
                }
                if (istart != 0) {
                    factor = (m_sample_ts[istart] - soft_fence.bound_ts.front()) / m_time_scale;
                    const scalar_type h{m_sample_ts[istart] - m_sample_ts[istart - 1]};
                    const scalar_type ratio{
                        (soft_fence.bound_ts.front() - m_sample_ts[istart - 1]) / h
                    };
                    const std::array<scalar_type, 4> proration_coeffs = prorationCoeffs(ratio, h);
                    m_osqp_problem.addRampCostTerm(
                        {{sIndex(istart - 1), -proration_coeffs[0]},
                         {sIndex(istart), -proration_coeffs[1]},
                         {vIndex(istart - 1), -proration_coeffs[2]},
                         {vIndex(istart), -proration_coeffs[3]}},
                        -soft_fence.bound_ss.front(), soft_fence.linear_weight * factor,
                        soft_fence.quadratic_weight * factor
                    );
                }
                for (index_type i{istart}; i < iend; ++i) {
                    if (i == istart) {
                        factor = m_hs[i] / m_time_scale * 0.5;
                    } else if (i == iend - 1) {
                        factor = m_hs[i - 1] / m_time_scale * 0.5;
                    } else {
                        factor = (m_hs[i - 1] + m_hs[i]) / m_time_scale * 0.5;
                    }
                    m_osqp_problem.addRampCostTerm(
                        {{sIndex(i), -1.0}}, -bound_func(m_sample_ts[i]),
                        soft_fence.linear_weight * factor, soft_fence.quadratic_weight * factor
                    );
                }
                if (iend != m_num_samples) {
                    factor = (soft_fence.bound_ts.back() - m_sample_ts[iend - 1]) / m_time_scale;
                    const scalar_type h{m_sample_ts[iend] - m_sample_ts[iend - 1]};
                    const scalar_type ratio{
                        (soft_fence.bound_ts.back() - m_sample_ts[iend - 1]) / h
                    };
                    const std::array<scalar_type, 4> proration_coeffs = prorationCoeffs(ratio, h);
                    m_osqp_problem.addRampCostTerm(
                        {{sIndex(iend - 1), -proration_coeffs[0]},
                         {sIndex(iend), -proration_coeffs[1]},
                         {vIndex(iend - 1), -proration_coeffs[2]},
                         {vIndex(iend), -proration_coeffs[3]}},
                        -soft_fence.bound_ss.back(), soft_fence.linear_weight * factor,
                        soft_fence.quadratic_weight * factor
                    );
                }
            } else {
                if (istart == iend) {
                    factor = (soft_fence.bound_ts.back() - soft_fence.bound_ts.front()) /
                             m_time_scale * 0.5;
                    const scalar_type h{m_sample_ts[istart] - m_sample_ts[istart - 1]};
                    scalar_type ratio{(soft_fence.bound_ts.front() - m_sample_ts[istart - 1]) / h};
                    std::array<scalar_type, 4> proration_coeffs = prorationCoeffs(ratio, h);
                    m_osqp_problem.addRampCostTerm(
                        {{sIndex(istart - 1), proration_coeffs[0]},
                         {sIndex(istart), proration_coeffs[1]},
                         {vIndex(istart - 1), proration_coeffs[2]},
                         {vIndex(istart), proration_coeffs[3]}},
                        soft_fence.bound_ss.front(), soft_fence.linear_weight * factor,
                        soft_fence.quadratic_weight * factor
                    );
                    ratio = (soft_fence.bound_ts.back() - m_sample_ts[iend - 1]) / h;
                    proration_coeffs = prorationCoeffs(ratio, h);
                    m_osqp_problem.addRampCostTerm(
                        {{sIndex(iend - 1), proration_coeffs[0]},
                         {sIndex(iend), proration_coeffs[1]},
                         {vIndex(iend - 1), proration_coeffs[2]},
                         {vIndex(iend), proration_coeffs[3]}},
                        soft_fence.bound_ss.back(), soft_fence.linear_weight * factor,
                        soft_fence.quadratic_weight * factor
                    );
                    continue;
                }
                if (istart != 0) {
                    factor = (m_sample_ts[istart] - soft_fence.bound_ts.front()) / m_time_scale;
                    const scalar_type h{m_sample_ts[istart] - m_sample_ts[istart - 1]};
                    const scalar_type ratio{
                        (soft_fence.bound_ts.front() - m_sample_ts[istart - 1]) / h
                    };
                    const std::array<scalar_type, 4> proration_coeffs = prorationCoeffs(ratio, h);
                    m_osqp_problem.addRampCostTerm(
                        {{sIndex(istart - 1), proration_coeffs[0]},
                         {sIndex(istart), proration_coeffs[1]},
                         {vIndex(istart - 1), proration_coeffs[2]},
                         {vIndex(istart), proration_coeffs[3]}},
                        soft_fence.bound_ss.front(), soft_fence.linear_weight * factor,
                        soft_fence.quadratic_weight * factor
                    );
                }
                for (index_type i{istart}; i < iend; ++i) {
                    if (i == istart) {
                        factor = m_hs[i] / m_time_scale * 0.5;
                    } else if (i == iend - 1) {
                        factor = m_hs[i - 1] / m_time_scale * 0.5;
                    } else {
                        factor = (m_hs[i - 1] + m_hs[i]) / m_time_scale * 0.5;
                    }
                    m_osqp_problem.addRampCostTerm(
                        {{sIndex(i), 1.0}}, bound_func(m_sample_ts[i]),
                        soft_fence.linear_weight * factor, soft_fence.quadratic_weight * factor
                    );
                }
                if (iend != m_num_samples) {
                    factor = (soft_fence.bound_ts.back() - m_sample_ts[iend - 1]) / m_time_scale;
                    const scalar_type h{m_sample_ts[iend] - m_sample_ts[iend - 1]};
                    const scalar_type ratio{
                        (soft_fence.bound_ts.back() - m_sample_ts[iend - 1]) / h
                    };
                    const std::array<scalar_type, 4> proration_coeffs = prorationCoeffs(ratio, h);
                    m_osqp_problem.addRampCostTerm(
                        {{sIndex(iend - 1), proration_coeffs[0]},
                         {sIndex(iend), proration_coeffs[1]},
                         {vIndex(iend - 1), proration_coeffs[2]},
                         {vIndex(iend), proration_coeffs[3]}},
                        soft_fence.bound_ss.back(), soft_fence.linear_weight * factor,
                        soft_fence.quadratic_weight * factor
                    );
                }
            }
        }
        return;
    }

    auto setVelocityRange(scalar_type lower_bound, scalar_type upper_bound) noexcept -> void {
        for (index_type i{1}; i < m_num_samples - 1; ++i) {
            m_osqp_problem.updateConstraintTerm(
                vIndex(i), {{vIndex(i), 1.0}}, lower_bound, upper_bound
            );
        }
        return;
    }

    auto setAccelRange(scalar_type lower_bound, scalar_type upper_bound) noexcept -> void {
        // m_osqp_problem.updateConstraintTerm(
        //     m_num_samples * 2,
        //     {{sIndex(0), -m_reciprocal_h2s[0] * 6.0},
        //      {sIndex(1), m_reciprocal_h2s[0] * 6.0},
        //      {vIndex(0), -m_reciprocal_hs[0] * 4.0},
        //      {vIndex(1), -m_reciprocal_hs[0] * 2.0}},
        //     lower_bound, upper_bound
        // );
        for (index_type i{1}; i < m_num_samples - 1; ++i) {
            m_osqp_problem.updateConstraintTerm(
                m_num_samples * 2 + i,
                {{sIndex(i), -m_reciprocal_h2s[i] * 6.0},
                 {sIndex(i + 1), m_reciprocal_h2s[i] * 6.0},
                 {vIndex(i), -m_reciprocal_hs[i] * 4.0},
                 {vIndex(i + 1), -m_reciprocal_hs[i] * 2.0}},
                lower_bound, upper_bound
            );
        }
        // m_osqp_problem.updateConstraintTerm(
        //     m_num_samples * 3 - 1,
        //     {{sIndex(m_num_samples - 2), m_reciprocal_h2s[m_num_samples - 2] * 6.0},
        //      {sIndex(m_num_samples - 1), -m_reciprocal_h2s[m_num_samples - 2] * 6.0},
        //      {vIndex(m_num_samples - 2), m_reciprocal_hs[m_num_samples - 2] * 2.0},
        //      {vIndex(m_num_samples - 1), m_reciprocal_hs[m_num_samples - 2] * 4.0}},
        //     lower_bound, upper_bound
        // );
        return;
    }

    auto setInitialState(
        scalar_type s0 = std::numeric_limits<scalar_type>::quiet_NaN(),
        scalar_type v0 = std::numeric_limits<scalar_type>::quiet_NaN(),
        scalar_type a0 = std::numeric_limits<scalar_type>::quiet_NaN()
    ) noexcept -> void {
        if (!std::isnan(s0)) {
            m_osqp_problem.updateConstraintTerm(
                sIndex(0), {{sIndex(0), 1.0}}, s0 - ::basjoo::math::kEpsilon,
                s0 + ::basjoo::math::kEpsilon
            );
        }
        if (!std::isnan(v0)) {
            m_osqp_problem.updateConstraintTerm(
                vIndex(0), {{vIndex(0), 1.0}}, v0 - ::basjoo::math::kEpsilon,
                v0 + ::basjoo::math::kEpsilon
            );
        }
        if (!std::isnan(a0)) {
            m_osqp_problem.updateConstraintTerm(
                m_num_samples * 3,
                {{sIndex(0), -m_reciprocal_h2s[0] * 6.0},
                 {sIndex(1), m_reciprocal_h2s[0] * 6.0},
                 {vIndex(0), -m_reciprocal_hs[0] * 4.0},
                 {vIndex(1), -m_reciprocal_hs[0] * 2.0}},
                a0 - ::basjoo::math::kEpsilon, a0 + ::basjoo::math::kEpsilon
            );
        }
        return;
    }

    auto setFinalState(
        scalar_type sf = std::numeric_limits<scalar_type>::quiet_NaN(),
        scalar_type vf = std::numeric_limits<scalar_type>::quiet_NaN(),
        scalar_type af = std::numeric_limits<scalar_type>::quiet_NaN()
    ) noexcept -> void {
        if (!std::isnan(sf)) {
            m_osqp_problem.updateConstraintTerm(
                sIndex(m_num_samples - 1), {{sIndex(m_num_samples - 1), 1.0}},
                sf - ::basjoo::math::kEpsilon, sf + ::basjoo::math::kEpsilon
            );
        }
        if (!std::isnan(vf)) {
            m_osqp_problem.updateConstraintTerm(
                vIndex(m_num_samples - 1), {{vIndex(m_num_samples - 1), 1.0}},
                vf - ::basjoo::math::kEpsilon, vf + ::basjoo::math::kEpsilon
            );
        }
        if (!std::isnan(af)) {
            m_osqp_problem.updateConstraintTerm(
                m_num_samples * 4 - 1,
                {{sIndex(m_num_samples - 2), m_reciprocal_h2s[m_num_samples - 2] * 6.0},
                 {sIndex(m_num_samples - 1), -m_reciprocal_h2s[m_num_samples - 2] * 6.0},
                 {vIndex(m_num_samples - 2), m_reciprocal_hs[m_num_samples - 2] * 2.0},
                 {vIndex(m_num_samples - 1), m_reciprocal_hs[m_num_samples - 2] * 4.0}},
                af - ::basjoo::math::kEpsilon, af + ::basjoo::math::kEpsilon
            );
        }
        return;
    }

    auto setVelocityCost(scalar_type target_velocity, scalar_type velocity_weight) noexcept
        -> void {
        constexpr std::array<scalar_type, 5> kFactors{
            6.0 / 5.0, 12.0 / 5.0, 1.0 / 5.0, 2.0 / 15.0, 1.0 / 15.0
        };
        for (index_type i{0}; i < m_num_samples - 1; ++i) {
            const scalar_type factor = velocity_weight * m_reciprocal_hs[i] / m_time_scale;
            m_osqp_problem.addQuadCostTerm(sIndex(i), sIndex(i), factor * kFactors[0]);
            m_osqp_problem.addQuadCostTerm(sIndex(i), sIndex(i + 1), -factor * kFactors[1]);
            m_osqp_problem.addQuadCostTerm(sIndex(i), vIndex(i), factor * m_hs[i] * kFactors[2]);
            m_osqp_problem.addQuadCostTerm(
                sIndex(i), vIndex(i + 1), factor * m_hs[i] * kFactors[2]
            );
            m_osqp_problem.addQuadCostTerm(sIndex(i + 1), sIndex(i + 1), factor * kFactors[0]);
            m_osqp_problem.addQuadCostTerm(
                sIndex(i + 1), vIndex(i), -factor * m_hs[i] * kFactors[2]
            );
            m_osqp_problem.addQuadCostTerm(
                sIndex(i + 1), vIndex(i + 1), -factor * m_hs[i] * kFactors[2]
            );
            m_osqp_problem.addQuadCostTerm(vIndex(i), vIndex(i), factor * m_h2s[i] * kFactors[3]);
            m_osqp_problem.addQuadCostTerm(
                vIndex(i), vIndex(i + 1), -factor * m_h2s[i] * kFactors[4]
            );
            m_osqp_problem.addQuadCostTerm(
                vIndex(i + 1), vIndex(i + 1), factor * m_h2s[i] * kFactors[3]
            );
            m_osqp_problem.addLinCostTerm(sIndex(i), factor * target_velocity * m_hs[i] * 2.0);
            m_osqp_problem.addLinCostTerm(sIndex(i + 1), -factor * target_velocity * m_hs[i] * 2.0);
        }
        return;
    }

    auto setAccelCost(scalar_type accel_weight) noexcept -> void {
        for (index_type i{0}; i < m_num_samples - 1; ++i) {
            const scalar_type factor =
                accel_weight * m_reciprocal_h2s[i] * m_reciprocal_hs[i] / m_time_scale;
            m_osqp_problem.addQuadCostTerm(sIndex(i), sIndex(i), factor * 12.0);
            m_osqp_problem.addQuadCostTerm(sIndex(i), sIndex(i + 1), -factor * 24.0);
            m_osqp_problem.addQuadCostTerm(sIndex(i), vIndex(i), factor * m_hs[i] * 12.0);
            m_osqp_problem.addQuadCostTerm(sIndex(i), vIndex(i + 1), factor * m_hs[i] * 12.0);
            m_osqp_problem.addQuadCostTerm(sIndex(i + 1), sIndex(i + 1), factor * 12.0);
            m_osqp_problem.addQuadCostTerm(sIndex(i + 1), vIndex(i), -factor * m_hs[i] * 12.0);
            m_osqp_problem.addQuadCostTerm(sIndex(i + 1), vIndex(i + 1), -factor * m_hs[i] * 12.0);
            m_osqp_problem.addQuadCostTerm(vIndex(i), vIndex(i), factor * m_h2s[i] * 4.0);
            m_osqp_problem.addQuadCostTerm(vIndex(i), vIndex(i + 1), factor * m_h2s[i] * 4.0);
            m_osqp_problem.addQuadCostTerm(vIndex(i + 1), vIndex(i + 1), factor * m_h2s[i] * 4.0);
        }
        return;
    }

    auto setJerkCost(scalar_type jerk_weight) noexcept -> void {
        for (index_type i{0}; i < m_num_samples - 1; ++i) {
            const scalar_type factor = jerk_weight * m_reciprocal_h2s[i] * m_reciprocal_h2s[i] *
                                       m_reciprocal_hs[i] / m_time_scale;
            m_osqp_problem.addQuadCostTerm(sIndex(i), sIndex(i), factor * 144.0);
            m_osqp_problem.addQuadCostTerm(sIndex(i), sIndex(i + 1), -factor * 288.0);
            m_osqp_problem.addQuadCostTerm(sIndex(i), vIndex(i), factor * m_hs[i] * 144.0);
            m_osqp_problem.addQuadCostTerm(sIndex(i), vIndex(i + 1), factor * m_hs[i] * 144.0);
            m_osqp_problem.addQuadCostTerm(sIndex(i + 1), sIndex(i + 1), factor * 144.0);
            m_osqp_problem.addQuadCostTerm(sIndex(i + 1), vIndex(i), -factor * m_hs[i] * 144.0);
            m_osqp_problem.addQuadCostTerm(sIndex(i + 1), vIndex(i + 1), -factor * m_hs[i] * 144.0);
            m_osqp_problem.addQuadCostTerm(vIndex(i), vIndex(i), factor * m_h2s[i] * 36.0);
            m_osqp_problem.addQuadCostTerm(vIndex(i), vIndex(i + 1), factor * m_h2s[i] * 72.0);
            m_osqp_problem.addQuadCostTerm(vIndex(i + 1), vIndex(i + 1), factor * m_h2s[i] * 36.0);
        }
        return;
    }

    [[using gnu: pure]]
    auto solve() const
        -> std::pair<Motion<scalar_type>, ::basjoo::cvxopm::Info<scalar_type, index_type>> {
        const ::basjoo::cvxopm::OsqpSolver<scalar_type, index_type> osqp_solver{m_settings};
        const auto [osqp_result, osqp_info] = osqp_solver.solve(m_osqp_problem);
        const scalar_type v0{osqp_result.prim_vars[vIndex(0)]};
        const scalar_type vf{osqp_result.prim_vars[vIndex(m_num_samples - 1)]};
        const scalar_type a0{
            -(osqp_result.prim_vars[sIndex(0)] - osqp_result.prim_vars[sIndex(1)]) *
                m_reciprocal_h2s[0] * 6.0 -
            (osqp_result.prim_vars[vIndex(0)] * 4.0 + osqp_result.prim_vars[vIndex(1)] * 2.0) *
                m_reciprocal_hs[0]
        };
        const scalar_type af = -(osqp_result.prim_vars[sIndex(m_num_samples - 1)] -
                                 osqp_result.prim_vars[sIndex(m_num_samples - 2)]) *
                                   m_reciprocal_h2s[m_num_samples - 2] * 6.0 +
                               (osqp_result.prim_vars[vIndex(m_num_samples - 1)] * 4.0 +
                                osqp_result.prim_vars[vIndex(m_num_samples - 2)] * 2.0) *
                                   m_reciprocal_hs[m_num_samples - 2];
        const std::array<::basjoo::math::BoundaryMode<value_type>, 2> b0{
            ::basjoo::math::BoundaryMode<value_type>{1, v0},
            ::basjoo::math::BoundaryMode<value_type>{2, a0}
        };
        const std::array<::basjoo::math::BoundaryMode<value_type>, 2> bf{
            ::basjoo::math::BoundaryMode<value_type>{1, vf},
            ::basjoo::math::BoundaryMode<value_type>{2, af}
        };
        std::pmr::vector<value_type> anchor_ss{
            osqp_result.prim_vars.data(),
            osqp_result.prim_vars.data() + sIndex(m_num_samples - 1) + 1, get_allocator()
        };

        return std::make_pair(
            Motion<scalar_type>{m_sample_ts, anchor_ss, b0, bf, get_allocator()}, osqp_info
        );
    }

  private:
    [[using gnu: const, always_inline, hot]]
    static constexpr auto prorationCoeffs(scalar_type ratio, scalar_type scale) noexcept
        -> std::array<scalar_type, 4> {
        const scalar_type ratio2 = ratio * ratio;
        const scalar_type ratio3 = ratio2 * ratio;
        return std::array<scalar_type, 4>{
            1.0 - ratio2 * 3.0 + ratio3 * 2.0, ratio2 * 3.0 - ratio3 * 2.0,
            (ratio - ratio2 * 2.0 + ratio3) * scale, -(ratio2 - ratio3) * scale
        };
    }

    auto setIntegrationRelation() noexcept -> void {
        for (index_type i{1}; i < m_num_samples - 1; ++i) {
            m_osqp_problem.updateConstraintTerm(
                m_num_samples * 3 + i,
                {{sIndex(i - 1), m_reciprocal_h2s[i - 1] * 3.0},
                 {sIndex(i), -(m_reciprocal_h2s[i - 1] - m_reciprocal_h2s[i]) * 3.0},
                 {sIndex(i + 1), -(m_reciprocal_h2s[i] * 3.0)},
                 {vIndex(i - 1), m_reciprocal_hs[i - 1]},
                 {vIndex(i), (m_reciprocal_hs[i - 1] + m_reciprocal_hs[i]) * 2.0},
                 {vIndex(i + 1), m_reciprocal_hs[i]}},
                -basjoo::math::kEpsilon, ::basjoo::math::kEpsilon
            );
        }
        return;
    }

    [[using gnu: pure, always_inline]]
    auto sIndex(index_type t_index) const noexcept -> index_type {
        return t_index;
    }

    [[using gnu: pure, always_inline]]
    auto vIndex(index_type t_index) const noexcept -> index_type {
        return m_num_samples + t_index;
    }

    index_type m_num_samples{0};
    scalar_type m_time_scale{0.0};
    ::basjoo::cvxopm::OsqpProblem<scalar_type, index_type> m_osqp_problem;
    ::basjoo::cvxopm::Settings<scalar_type, index_type> m_settings;
    std::pmr::vector<scalar_type> m_sample_ts;
    std::pmr::vector<scalar_type> m_hs;
    std::pmr::vector<scalar_type> m_h2s;
    std::pmr::vector<scalar_type> m_reciprocal_hs;
    std::pmr::vector<scalar_type> m_reciprocal_h2s;
};

} // namespace basjoo::bicycle
