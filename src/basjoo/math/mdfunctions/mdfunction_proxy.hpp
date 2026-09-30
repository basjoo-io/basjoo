/**
 * @file mdfunction_proxy.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-09-07
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <cstdint>
#include <system_error>
#include <utility>

#include "proxy/proxy.h"
#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/matrixx.hpp"
#include "basjoo/math/mdfunctions/linear_mdfunction.hpp"
#include "basjoo/math/mdfunctions/quadratic_mdfunction.hpp"
#include "basjoo/math/mdfunctions/rosenbrock_function.hpp"

namespace basjoo::math {

namespace detail {

// NOLINTBEGIN(modernize-use-trailing-return-type)

PRO_DEF_MEM_DISPATCH(c_num_dimensions, num_dimensions);
PRO_DEF_MEM_DISPATCH(c_eval, eval);
PRO_DEF_MEM_DISPATCH(c_gradient, gradient);
PRO_DEF_MEM_DISPATCH(c_has_extrema, has_extrema);

// NOLINTEND(modernize-use-trailing-return-type)

// clang-format off

template <VecArithmetic T>
class MdFunctionFacade final : public pro::skills::rtti<typename pro::facade_builder
    ::support_copy<pro::constraint_level::nontrivial>
    ::add_convention<c_num_dimensions, auto() const noexcept -> typename T::size_type>
    ::template add_convention<pro::operator_dispatch<"()">, auto(const T&) const noexcept -> typename T::value_type>
    ::template add_convention<c_eval, auto(const T&) const noexcept -> typename T::value_type>
    ::template add_convention<c_gradient, auto(const T&) const noexcept -> T>
    ::template add_convention<c_gradient, auto(const T&, std::size_t) const noexcept -> typename T::value_type>
    ::template add_convention<c_has_extrema, auto(const T&) const noexcept -> bool>
    >::build {};

// clang-format on

enum class MdFunctionProxyTag : std::uint8_t {
    LINEAR,
    QUADRATIC,
    ROSENBROCK
};

template <VecArithmetic T>
constexpr auto serialize(auto& archive, const pro::proxy<MdFunctionFacade<T>>& self)
    -> zpp::bits::errc {
    if (const auto* mdfunction = proxy_cast<LinearMdFunction<T>>(&*self)) {
        if (auto result = archive(MdFunctionProxyTag::LINEAR); zpp::bits::failure(result))
            [[unlikely]] {
            return result;
        }
        return archive(*mdfunction);
    }
    if (const auto* mdfunction = proxy_cast<QuadraticMdFunction<T>>(&*self)) {
        if (auto result = archive(MdFunctionProxyTag::QUADRATIC); zpp::bits::failure(result))
            [[unlikely]] {
            return result;
        }
        return archive(*mdfunction);
    }
    if (const auto* mdfunction = proxy_cast<RosenbrockFunction<T>>(&*self)) {
        if (auto result = archive(MdFunctionProxyTag::ROSENBROCK); zpp::bits::failure(result))
            [[unlikely]] {
            return result;
        }
        return archive(*mdfunction);
    }
    return std::errc::invalid_argument;
}

template <VecArithmetic T>
constexpr auto serialize(auto& archive, pro::proxy<MdFunctionFacade<T>>& self) -> zpp::bits::errc {
    using archive_type = std::remove_cvref_t<decltype(archive)>;
    if constexpr (archive_type::kind() == zpp::bits::kind::out) {
        if (auto* mdfunction = proxy_cast<LinearMdFunction<T>>(&*self)) {
            if (auto result = archive(MdFunctionProxyTag::LINEAR); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*mdfunction);
        }
        if (auto* mdfunction = proxy_cast<QuadraticMdFunction<T>>(&*self)) {
            if (auto result = archive(MdFunctionProxyTag::QUADRATIC); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*mdfunction);
        }
        if (auto* mdfunction = proxy_cast<RosenbrockFunction<T>>(&*self)) {
            if (auto result = archive(MdFunctionProxyTag::ROSENBROCK); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*mdfunction);
        }
        return std::errc::invalid_argument;
    } else {
        MdFunctionProxyTag tag{};
        if (auto result = archive(tag); zpp::bits::failure(result)) [[unlikely]] {
            return result;
        }
        switch (tag) {
        case MdFunctionProxyTag::LINEAR: {
            LinearMdFunction<T> mdfunction{};
            if (auto result = archive(mdfunction); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<MdFunctionFacade<T>>(std::move(mdfunction));
            return {};
        }
        case MdFunctionProxyTag::QUADRATIC: {
            QuadraticMdFunction<T> mdfunction{};
            if (auto result = archive(mdfunction); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<MdFunctionFacade<T>>(std::move(mdfunction));
            return {};
        }
        case MdFunctionProxyTag::ROSENBROCK: {
            RosenbrockFunction<T> mdfunction{};
            if (auto result = archive(mdfunction); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<MdFunctionFacade<T>>(std::move(mdfunction));
            return {};
        }
        }
        return std::errc::invalid_argument;
    }
}

} // namespace detail

template <VecArithmetic T>
using MdFunctionProxy = pro::proxy<detail::MdFunctionFacade<T>>;

template <typename T>
[[using gnu: always_inline]]
inline auto makeMdFunctionProxy(T&& mdfunction)
    -> MdFunctionProxy<typename std::remove_cvref_t<T>::param_type> {
    return pro::make_proxy<detail::MdFunctionFacade<typename std::remove_cvref_t<T>::param_type>>(
        std::forward<T>(mdfunction)
    );
}

} // namespace basjoo::math
