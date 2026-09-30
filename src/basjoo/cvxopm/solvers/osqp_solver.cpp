/**
 * @file osqp_solver.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-11-03
 *
 * @copyright Copyright (c) 2024 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/cvxopm/solvers/osqp_solver.hpp"

#include <format>
#include <memory>
#include <span>
#include <stdexcept>

#include "osqp.h"

#include "basjoo/math/sparse/csc_matrix.hpp"

namespace basjoo::cvxopm {

template <std::floating_point Scalar, std::integral Index>
[[using gnu: pure, visibility("default")]] [[nodiscard]]
auto OsqpSolver<Scalar, Index>::solve(
    const OsqpProblem<Scalar, Index>& osqp_problem, std::span<const Scalar> prim_vars_0,
    std::span<const Scalar> dual_vars_0
) const -> std::pair<::basjoo::cvxopm::Result<Scalar>, Info<Scalar, Index>> {
#if BASJOO_CHECK_PARAMS == 1
    if (!prim_vars_0.empty() && prim_vars_0.size() != osqp_problem.num_variables()) [[unlikely]] {
        throw std::invalid_argument(
            std::format(
                "Osqp runtime error detected! Size of prim_vars_0_ and num_vars must be identical: "
                "prim_vars_0_.size() = {0:d} while num_vars = {1:d}",
                prim_vars_0.size(), osqp_problem.num_variables()
            )
        );
    }
    if (!dual_vars_0.empty() && dual_vars_0.size() != osqp_problem.num_constraints()) [[unlikely]] {
        throw std::invalid_argument(
            std::format(
                "Osqp runtime error detected! Size of dual_vars_0_ and num_cons must be identical: "
                "dual_vars_0_.size() = {0:d} while num_cons = {1:d}",
                dual_vars_0.size(), osqp_problem.num_constraints()
            )
        );
    }
#endif

    OSQPSettings osqp_settings{
        .device = static_cast<Index>(settings.device),
        .linsys_solver = static_cast<enum osqp_linsys_solver_type>(settings.linsys_solver),
        .allocate_solution = static_cast<Index>(settings.allocate_solution),
        .verbose = static_cast<Index>(settings.verbose),
        .profiler_level = static_cast<Index>(settings.profiler_level),
        .warm_starting = static_cast<Index>(settings.warm_starting),
        .scaling = static_cast<Index>(settings.scaling),
        .polishing = static_cast<Index>(settings.polishing),
        .rho = static_cast<Scalar>(settings.rho),
        .rho_is_vec = static_cast<Index>(settings.rho_is_vec),
        .sigma = static_cast<Scalar>(settings.sigma),
        .alpha = static_cast<Scalar>(settings.alpha),
        .cg_max_iter = static_cast<Index>(settings.cg_max_iter),
        .cg_tol_reduction = static_cast<Index>(settings.cg_tol_reduction),
        .cg_tol_fraction = static_cast<Scalar>(settings.cg_tol_fraction),
        .cg_precond = static_cast<osqp_precond_type>(settings.cg_precond),
        .adaptive_rho = static_cast<Index>(settings.adaptive_rho),
        .adaptive_rho_interval = static_cast<Index>(settings.adaptive_rho_interval),
        .adaptive_rho_fraction = static_cast<Scalar>(settings.adaptive_rho_fraction),
        .adaptive_rho_tolerance = static_cast<Scalar>(settings.adaptive_rho_tolerance),
        .max_iter = static_cast<Index>(settings.max_iter),
        .eps_abs = static_cast<Scalar>(settings.eps_abs),
        .eps_rel = static_cast<Scalar>(settings.eps_rel),
        .eps_prim_inf = static_cast<Scalar>(settings.eps_prim_inf),
        .eps_dual_inf = static_cast<Scalar>(settings.eps_dual_inf),
        .scaled_termination = static_cast<Index>(settings.scaled_termination),
        .check_termination = static_cast<Index>(settings.check_termination),
        .check_dualgap = static_cast<Index>(settings.check_dualgap),
        .time_limit = static_cast<Scalar>(settings.time_limit),
        .delta = static_cast<Scalar>(settings.delta),
        .polish_refine_iter = static_cast<Index>(settings.polish_refine_iter)
    };

    const auto num_vars{static_cast<Index>(osqp_problem.num_variables())};
    const auto num_cons{static_cast<Index>(osqp_problem.num_constraints())};
    const ::basjoo::math::pmr::CscMatrix<Scalar, Index> objective_matrix{
        osqp_problem.m_objective_matrix
    };
    const ::basjoo::math::pmr::CscMatrix<Scalar, Index> constraint_matrix{
        osqp_problem.m_constraint_matrix
    };

    const OSQPCscMatrix P{
        .m = num_vars,
        .n = num_vars,
        .p = const_cast<Index*>(objective_matrix.outerIndices().data()),
        .i = const_cast<Index*>(objective_matrix.innerIndices().data()),
        .x = const_cast<Scalar*>(objective_matrix.values().data()),
        .nzmax = static_cast<Index>(objective_matrix.nnzs()),
        .nz = -1,
        .owned = 0
    };

    const OSQPCscMatrix A{
        .m = num_cons,
        .n = num_vars,
        .p = const_cast<Index*>(constraint_matrix.outerIndices().data()),
        .i = const_cast<Index*>(constraint_matrix.innerIndices().data()),
        .x = const_cast<Scalar*>(constraint_matrix.values().data()),
        .nzmax = static_cast<Index>(constraint_matrix.nnzs()),
        .nz = -1,
        .owned = 0
    };

    std::unique_ptr<OSQPSolver, decltype(&osqp_cleanup)> solver{nullptr, osqp_cleanup};

    Index exit_flag = osqp_setup(
        std::inout_ptr(solver), &P, osqp_problem.m_objective_vector.data(), &A,
        osqp_problem.m_lower_bounds.data(), osqp_problem.m_upper_bounds.data(), num_cons, num_vars,
        &osqp_settings
    );

    if (exit_flag != 0) [[unlikely]] {
        throw std::runtime_error(
            std::format(
                "Osqp runtime error detected! Not able to setup a OSQPSolver: exit_flag = {0:d}.",
                exit_flag
            )
        );
    }

    // Set warm start
    if (!(prim_vars_0.empty() && dual_vars_0.empty())) {
        exit_flag = osqp_warm_start(
            solver.get(), prim_vars_0.empty() ? nullptr : prim_vars_0.data(),
            dual_vars_0.empty() ? nullptr : dual_vars_0.data()
        );
        if (exit_flag != 0) [[unlikely]] {
            throw std::runtime_error(
                std::format(
                    "Osqp runtime error detected! Not able to set warm start for a OSQPSolver: "
                    "exit_flag = {0:d}.",
                    exit_flag
                )
            );
        }
    }

    exit_flag = osqp_solve(solver.get());
    if (exit_flag != 0) [[unlikely]] {
        throw std::runtime_error(
            std::format(
                "Osqp runtime error detected! Not able to execute a OSQPSolver: exit_flag = {0:d}.",
                exit_flag
            )
        );
    }

    ::basjoo::cvxopm::Result<Scalar> result{
        .prim_vars{solver->solution->x, solver->solution->x + num_vars, settings.memory_resource},
        .dual_vars{solver->solution->y, solver->solution->y + num_cons, settings.memory_resource},
        .prim_inf_cert{
            solver->solution->prim_inf_cert, solver->solution->prim_inf_cert + num_cons,
            settings.memory_resource
        },
        .dual_inf_cert{
            solver->solution->dual_inf_cert, solver->solution->dual_inf_cert + num_vars,
            settings.memory_resource
        },
    };

    ::basjoo::cvxopm::Info<Scalar, Index> info{
        .status{std::to_array(solver->info->status)},
        .status_val = solver->info->status_val,
        .status_polish = solver->info->status_polish,
        .obj_val = solver->info->obj_val,
        .dual_obj_val = solver->info->dual_obj_val,
        .prim_res = solver->info->prim_res,
        .dual_res = solver->info->dual_res,
        .duality_gap = solver->info->duality_gap,
        .iter = solver->info->iter,
        .rho_updates = solver->info->rho_updates,
        .rho_estimate = solver->info->rho_estimate,
        .setup_time = solver->info->setup_time,
        .solve_time = solver->info->solve_time,
        .update_time = solver->info->update_time,
        .polish_time = solver->info->polish_time,
        .run_time = solver->info->run_time,
        .primdual_int = solver->info->primdual_int,
        .rel_kkt_error = solver->info->rel_kkt_error,
    };

    return std::make_pair(std::move(result), info);
}

template auto OsqpSolver<OSQPFloat, OSQPInt>::solve(
    const OsqpProblem<OSQPFloat, OSQPInt>& osqp_problem, std::span<const OSQPFloat> prim_vars_0,
    std::span<const OSQPFloat> dual_vars_0
) const -> std::pair<::basjoo::cvxopm::Result<OSQPFloat>, Info<OSQPFloat, OSQPInt>>;

} // namespace basjoo::cvxopm
