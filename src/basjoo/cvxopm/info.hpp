/**
 * @file info.hpp
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

#include <array>
#include <concepts>

#include "zpp_bits.h"

namespace basjoo::cvxopm {

template <std::floating_point Scalar, std::integral Index = int>
struct Info final {
    using serialize = zpp::bits::members<18>;

    using value_type = Scalar;
    using index_type = Index;

    // solver status
    std::array<char, 32> status; // Status string, e.g. 'solved'
    index_type status_val;       // Status as Index.
    index_type
        status_polish; // Polishing status: successful (1), unperformed (0), unsuccessful (-1)

    // solution quality
    value_type obj_val;      // Primal objective v
    value_type dual_obj_val; ///< Dual objective value
    value_type prim_res;     // Norm of primal residual
    value_type dual_res;     // Norm of dual residual
    value_type duality_gap;  ///< Duality gap (Primal obj - Dual obj)

    // algorithm information
    index_type iter;         // Number of iterations taken
    index_type rho_updates;  // Number of rho updates performned
    value_type rho_estimate; // Best rho estimate so far from residuals

    // timing information
    value_type setup_time;  // Setup phase time (seconds)
    value_type solve_time;  // Solve phase time (seconds)
    value_type update_time; // Update phase time (seconds)
    value_type polish_time; // Polish phase time (seconds)
    value_type run_time;    // Total solve time (seconds)

    // Convergence information
    value_type primdual_int; ///< Integral of duality gap over time (Primal-dual integral), requires
                             ///< profiling
    value_type rel_kkt_error; ///< Relative KKT error
};

} // namespace basjoo::cvxopm
