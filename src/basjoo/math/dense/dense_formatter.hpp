/**
 * @file dense_formatter.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief std::format and std::ostream support for dense vectors and matrices.
 * @version 0.1
 * @date 2026-10-14
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <cstddef>
#include <format>
#include <ostream>

#include "basjoo/math/complex_formatter.hpp" // std::formatter for complex elements
#include "basjoo/math/concepts.hpp"

namespace basjoo::math {

// Same text as std::format("{}", vector).
template <typename Char>
inline auto operator<<(std::basic_ostream<Char>& os, const VecArithmetic auto& vector)
    -> std::basic_ostream<Char>& {
    if constexpr (std::same_as<Char, wchar_t>) {
        os << std::format(L"{}", vector);
    } else {
        os << std::format("{}", vector);
    }
    return os;
}

// Same text as std::format("{}", matrix).
template <typename Char>
inline auto operator<<(std::basic_ostream<Char>& os, const MatArithmetic auto& matrix)
    -> std::basic_ostream<Char>& {
    if constexpr (std::same_as<Char, wchar_t>) {
        os << std::format(L"{}", matrix);
    } else {
        os << std::format("{}", matrix);
    }
    return os;
}

} // namespace basjoo::math

// NOLINTBEGIN(cert-dcl58-cpp)
namespace std {

// Vectors are not ranges, so the standard range formatter does not compete. Output: "[a, b, c]".
template <::basjoo::math::VecArithmetic V, typename Char>
struct formatter<V, Char> {
    constexpr auto parse(std::basic_format_parse_context<Char>& ctx)
        -> std::basic_format_parse_context<Char>::iterator {
        return m_element.parse(ctx);
    }

    template <typename FormatContext>
    auto format(const V& vector, FormatContext& ctx) const -> FormatContext::iterator {
        auto out{ctx.out()};
        *out++ = static_cast<Char>('[');
        for (std::size_t i{0}; i < vector.size(); ++i) {
            if (i != 0) {
                *out++ = static_cast<Char>(',');
                *out++ = static_cast<Char>(' ');
            }
            ctx.advance_to(out);
            out = m_element.format(vector.data()[i], ctx);
        }
        *out++ = static_cast<Char>(']');
        return out;
    }

  private:
    std::formatter<typename V::value_type, Char> m_element;
};

// Row by row regardless of the storage order. Output: "[[a, b], [c, d]]".
template <::basjoo::math::MatArithmetic M, typename Char>
struct formatter<M, Char> {
    constexpr auto parse(std::basic_format_parse_context<Char>& ctx)
        -> std::basic_format_parse_context<Char>::iterator {
        return m_element.parse(ctx);
    }

    template <typename FormatContext>
    auto format(const M& matrix, FormatContext& ctx) const -> FormatContext::iterator {
        auto out{ctx.out()};
        *out++ = static_cast<Char>('[');
        for (typename M::index_type i{0}; i < matrix.nrows(); ++i) {
            if (i != 0) {
                *out++ = static_cast<Char>(',');
                *out++ = static_cast<Char>(' ');
            }
            *out++ = static_cast<Char>('[');
            for (typename M::index_type j{0}; j < matrix.ncols(); ++j) {
                if (j != 0) {
                    *out++ = static_cast<Char>(',');
                    *out++ = static_cast<Char>(' ');
                }
                ctx.advance_to(out);
                out = m_element.format(matrix[i, j], ctx);
            }
            *out++ = static_cast<Char>(']');
        }
        *out++ = static_cast<Char>(']');
        return out;
    }

  private:
    std::formatter<typename M::value_type, Char> m_element;
};

} // namespace std
// NOLINTEND(cert-dcl58-cpp)
