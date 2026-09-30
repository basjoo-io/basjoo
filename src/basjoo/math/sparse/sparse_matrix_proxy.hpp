/**
 * @file sparse_matrix_proxy.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-27
 *
 * @copyright Copyright (c) 2024 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <cstdint>
#include <system_error>
#include <utility>

#include "proxy/proxy.h"
#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/sparse/coo_matrix.hpp"
#include "basjoo/math/sparse/csc_matrix.hpp"
#include "basjoo/math/sparse/csr_matrix.hpp"
#include "basjoo/math/sparse/dok_matrix.hpp"
#include "basjoo/math/sparse/lil_matrix.hpp"

namespace basjoo::math {

namespace detail {

// NOLINTBEGIN(modernize-use-trailing-return-type)

PRO_DEF_MEM_DISPATCH(c_nrows, nrows);
PRO_DEF_MEM_DISPATCH(c_ncols, ncols);
PRO_DEF_MEM_DISPATCH(c_nnzs, nnzs);
PRO_DEF_MEM_DISPATCH(m_resize, resize);
PRO_DEF_MEM_DISPATCH(m_reserve, reserve);
PRO_DEF_MEM_DISPATCH(m_clear, clear);
PRO_DEF_MEM_DISPATCH(m_compress, compress);
PRO_DEF_MEM_DISPATCH(c_coeff, coeff);
PRO_DEF_MEM_DISPATCH(m_updateCoeff, updateCoeff);

// NOLINTEND(modernize-use-trailing-return-type)

// clang-format off

template <::basjoo::math::ScalarArithmetic Scalar, std::integral Index>
class SparseMatrixFacade final : public pro::skills::rtti<typename pro::facade_builder
    ::support_copy<pro::constraint_level::nontrivial>
    ::add_convention<c_nrows, auto() const noexcept -> std::size_t>
    ::add_convention<c_ncols, auto() const noexcept -> std::size_t>
    ::add_convention<c_nnzs, auto() const noexcept -> std::size_t>
    ::add_convention<m_resize, auto(std::size_t, std::size_t) -> void>
    ::add_convention<m_reserve, auto(std::size_t) -> void>
    ::add_convention<m_clear, auto() -> void>
    ::add_convention<m_compress, auto() -> void>
    ::template add_convention<c_coeff, auto(std::size_t, std::size_t) const noexcept -> Scalar>
    ::template add_convention<m_updateCoeff, auto(std::size_t, std::size_t, const Scalar&) -> void>
    ::template add_convention<pro::operator_dispatch<"[]">, auto(std::size_t, std::size_t) const noexcept -> Scalar>
    >::build {};

// clang-format on

enum class SparseMatrixProxyTag : std::uint8_t {
    DOK,
    LIL,
    COO,
    CSC,
    CSR
};

template <ScalarArithmetic Scalar, std::integral Index>
constexpr auto serialize(auto& archive, const pro::proxy<SparseMatrixFacade<Scalar, Index>>& self)
    -> zpp::bits::errc {
    if (const auto* sparse_matrix = proxy_cast<DokMatrix<Scalar, Index>>(&*self)) {
        if (auto result = archive(SparseMatrixProxyTag::DOK); zpp::bits::failure(result))
            [[unlikely]] {
            return result;
        }
        return archive(*sparse_matrix);
    }
    if (const auto* sparse_matrix = proxy_cast<LilMatrix<Scalar, Index>>(&*self)) {
        if (auto result = archive(SparseMatrixProxyTag::LIL); zpp::bits::failure(result))
            [[unlikely]] {
            return result;
        }
        return archive(*sparse_matrix);
    }
    if (const auto* sparse_matrix = proxy_cast<CooMatrix<Scalar, Index>>(&*self)) {
        if (auto result = archive(SparseMatrixProxyTag::COO); zpp::bits::failure(result))
            [[unlikely]] {
            return result;
        }
        return archive(*sparse_matrix);
    }
    if (const auto* sparse_matrix = proxy_cast<CscMatrix<Scalar, Index>>(&*self)) {
        if (auto result = archive(SparseMatrixProxyTag::CSC); zpp::bits::failure(result))
            [[unlikely]] {
            return result;
        }
        return archive(*sparse_matrix);
    }
    if (const auto* sparse_matrix = proxy_cast<CsrMatrix<Scalar, Index>>(&*self)) {
        if (auto result = archive(SparseMatrixProxyTag::CSR); zpp::bits::failure(result))
            [[unlikely]] {
            return result;
        }
        return archive(*sparse_matrix);
    }
    return std::errc::invalid_argument;
}

template <ScalarArithmetic Scalar, std::integral Index>
constexpr auto serialize(auto& archive, pro::proxy<SparseMatrixFacade<Scalar, Index>>& self)
    -> zpp::bits::errc {
    using archive_type = std::remove_cvref_t<decltype(archive)>;
    if constexpr (archive_type::kind() == zpp::bits::kind::out) {
        if (auto* sparse_matrix = proxy_cast<DokMatrix<Scalar, Index>>(&*self)) {
            if (auto result = archive(SparseMatrixProxyTag::DOK); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*sparse_matrix);
        }
        if (auto* sparse_matrix = proxy_cast<LilMatrix<Scalar, Index>>(&*self)) {
            if (auto result = archive(SparseMatrixProxyTag::LIL); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*sparse_matrix);
        }
        if (auto* sparse_matrix = proxy_cast<CooMatrix<Scalar, Index>>(&*self)) {
            if (auto result = archive(SparseMatrixProxyTag::COO); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*sparse_matrix);
        }
        if (auto* sparse_matrix = proxy_cast<CscMatrix<Scalar, Index>>(&*self)) {
            if (auto result = archive(SparseMatrixProxyTag::CSC); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*sparse_matrix);
        }
        if (auto* sparse_matrix = proxy_cast<CsrMatrix<Scalar, Index>>(&*self)) {
            if (auto result = archive(SparseMatrixProxyTag::CSR); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*sparse_matrix);
        }
        return std::errc::invalid_argument;
    } else {
        SparseMatrixProxyTag tag{};
        if (auto result = archive(tag); zpp::bits::failure(result)) [[unlikely]] {
            return result;
        }
        switch (tag) {
        case SparseMatrixProxyTag::DOK: {
            DokMatrix<Scalar, Index> sparse_matrix{};
            if (auto result = archive(sparse_matrix); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<SparseMatrixFacade<Scalar, Index>>(std::move(sparse_matrix));
            return {};
        }
        case SparseMatrixProxyTag::LIL: {
            LilMatrix<Scalar, Index> sparse_matrix{};
            if (auto result = archive(sparse_matrix); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<SparseMatrixFacade<Scalar, Index>>(std::move(sparse_matrix));
            return {};
        }
        case SparseMatrixProxyTag::COO: {
            CooMatrix<Scalar, Index> sparse_matrix{};
            if (auto result = archive(sparse_matrix); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<SparseMatrixFacade<Scalar, Index>>(std::move(sparse_matrix));
            return {};
        }
        case SparseMatrixProxyTag::CSC: {
            CscMatrix<Scalar, Index> sparse_matrix{};
            if (auto result = archive(sparse_matrix); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<SparseMatrixFacade<Scalar, Index>>(std::move(sparse_matrix));
            return {};
        }
        case SparseMatrixProxyTag::CSR: {
            CsrMatrix<Scalar, Index> sparse_matrix{};
            if (auto result = archive(sparse_matrix); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<SparseMatrixFacade<Scalar, Index>>(std::move(sparse_matrix));
            return {};
        }
        }
        return std::errc::invalid_argument;
    }
}

} // namespace detail

template <ScalarArithmetic Scalar, std::integral Index>
using SparseMatrixProxy = pro::proxy<detail::SparseMatrixFacade<Scalar, Index>>;

template <typename T>
[[using gnu: always_inline]]
inline auto makeSparseMatrixProxy(T&& sparse_matrix) -> SparseMatrixProxy<
    typename std::remove_cvref_t<T>::value_type, typename std::remove_cvref_t<T>::index_type> {
    return pro::make_proxy<detail::SparseMatrixFacade<
        typename std::remove_cvref_t<T>::value_type, typename std::remove_cvref_t<T>::index_type>>(
        std::forward<T>(sparse_matrix)
    );
}

} // namespace basjoo::math
