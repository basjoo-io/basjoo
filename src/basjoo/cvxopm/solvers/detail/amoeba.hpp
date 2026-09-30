/**
 * @file amoeba.hpp
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
#include <list>
#include <memory_resource>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/mdfunctions/mdfunction_proxy.hpp"

namespace basjoo::cvxopm::detail {

template <::basjoo::math::VecArithmetic T>
class Amoeba final {
  public:
    struct VertexNode final {
        [[using gnu: pure, always_inline]]
        constexpr auto operator<(const VertexNode& obj) const noexcept -> bool {
            return f < obj.f;
        }

        typename T::value_type f{0.0};
        T x;
    };

    using param_type = T;
    using scalar_type = typename param_type::value_type;
    using size_type = typename param_type::size_type;
    using value_type = VertexNode;
    using allocator_type = std::pmr::polymorphic_allocator<value_type>;

    Amoeba() noexcept = default;
    Amoeba(const Amoeba& other) noexcept = delete;
    auto operator=(const Amoeba& other) noexcept -> Amoeba& = delete;
    Amoeba(Amoeba&& other) noexcept = delete;
    auto operator=(Amoeba&& other) noexcept -> Amoeba& = delete;
    ~Amoeba() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_vertex_nodes.get_allocator();
    }

    [[using gnu: always_inline]]
    explicit Amoeba(
        const ::basjoo::math::MdFunctionProxy<param_type>& objective_function, const param_type& x0,
        scalar_type step = 1.0, const allocator_type& alloc = {}
    ) noexcept
        : m_objective_function(objective_function), m_vertex_nodes(alloc),
          m_vertices_sum(x0.size(), 0.0, alloc) {
        scalar_type f{objective_function->eval(x0)};
        m_vertex_nodes.emplace_back(VertexNode{f, x0});
        for (size_type i{0}; i < x0.size(); ++i) {
            param_type vertex{x0};
            vertex[i] += step;
            scalar_type f{objective_function->eval(vertex)};
            m_vertex_nodes.emplace_back(VertexNode{f, std::move(vertex)});
        }
        m_vertex_nodes.sort();
        for (const auto& [f, x] : m_vertex_nodes) {
            m_vertices_sum += x;
        }
    }

    [[using gnu: flatten, leaf]]
    auto contract(scalar_type ratio) noexcept -> void {
        for (auto it = std::next(m_vertex_nodes.begin()); it != m_vertex_nodes.end(); ++it) {
            std::transform(
                m_vertex_nodes.cbegin()->x.data(),
                m_vertex_nodes.cbegin()->x.data() + m_vertex_nodes.cbegin()->x.size(), it->x.data(),
                it->x.data(),
                [ratio](const scalar_type& a, const scalar_type& b) noexcept -> scalar_type {
                    return (1 - ratio) * a + ratio * b;
                }
            );
            it->f = m_objective_function->eval(it->x);
        }
        m_vertex_nodes.sort();
        m_vertices_sum.fill(0.0);
        for (const auto& [f, x] : m_vertex_nodes) {
            m_vertices_sum += x;
        }
        return;
    }

    [[using gnu: flatten, leaf]]
    auto lerpTopVertex(scalar_type ratio) noexcept -> VertexNode {
        const scalar_type fac1{
            (static_cast<scalar_type>(1.0) - ratio) /
            static_cast<scalar_type>(m_vertices_sum.size())
        };
        const scalar_type fac2{fac1 - ratio};
        param_type x = m_vertices_sum * fac1 - m_vertex_nodes.back().x * fac2;
        scalar_type f = m_objective_function->eval(x);
        VertexNode new_vertex_node{f, std::move(x)};
        // if the new vertex is better than the worst vertex, replace it
        if (f < m_vertex_nodes.back().f) {
            update(new_vertex_node);
        }
        return new_vertex_node;
    };

    [[using gnu: pure, always_inline]]
    auto vertex_nodes() const noexcept -> const std::pmr::list<VertexNode>& {
        return m_vertex_nodes;
    }

    [[using gnu: pure, always_inline]]
    auto vertices_sum() const noexcept -> const param_type& {
        return m_vertices_sum;
    }

    [[using gnu: pure, always_inline]]
    auto converged(scalar_type tol) const noexcept -> bool {
        scalar_type fmin{m_vertex_nodes.front().f};
        scalar_type fmax{m_vertex_nodes.back().f};
        return std::abs(fmax - fmin) / (std::abs(fmax) + std::abs(fmin)) * 2.0 < tol;
    }

  private:
    [[using gnu: always_inline]]
    auto update(VertexNode vertex_node) noexcept -> void {
        m_vertices_sum -= m_vertex_nodes.back().x;
        m_vertex_nodes.pop_back();
        m_vertices_sum += vertex_node.x;
        auto pos = std::upper_bound(m_vertex_nodes.cbegin(), m_vertex_nodes.cend(), vertex_node);
        m_vertex_nodes.emplace(pos, std::move(vertex_node));
        return;
    }

    ::basjoo::math::MdFunctionProxy<param_type> m_objective_function;
    std::list<VertexNode, allocator_type> m_vertex_nodes;
    param_type m_vertices_sum;
};

} // namespace basjoo::cvxopm::detail
