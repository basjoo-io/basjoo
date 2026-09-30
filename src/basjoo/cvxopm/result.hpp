/**
 * @file result.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-11-03
 *
 * @copyright Copyright (c) 2024 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <vector>

#include "zpp_bits.h"

namespace basjoo::cvxopm {

template <std::floating_point Scalar>
struct Result final {
    using serialize = zpp::bits::members<4>;

    using value_type = Scalar;
    using param_type = std::pmr::vector<value_type>;
    using allocator_type = typename param_type::allocator_type;

    [[using gnu: always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return prim_vars.get_allocator();
    }

    param_type prim_vars;
    param_type dual_vars;
    param_type prim_inf_cert;
    param_type dual_inf_cert;
};

} // namespace basjoo::cvxopm
