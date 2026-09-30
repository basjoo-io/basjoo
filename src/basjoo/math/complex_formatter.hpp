/**
 * @file complex_formatter.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief std::format support for complex scalars.
 * @version 0.1
 * @date 2026-10-13
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <format>
#include <string>
#include <string_view>

#include "basjoo/math/concepts.hpp"

// NOLINTBEGIN(cert-dcl58-cpp)
namespace std {

// Output: "(re+imi)". The std-format-spec, e.g. "{:.3f}" or "{:>10.2e}", applies to each part and
// the imaginary sign is always shown, like C++26's std::formatter<std::complex>.
// TODO(houchen_li): Remove once the standard library ships its own std::formatter<std::complex>
// (P2197), which would otherwise clash with this one
template <::basjoo::math::ComplexArithmetic T, typename Char>
struct formatter<T, Char> {
    constexpr auto parse(std::basic_format_parse_context<Char>& ctx)
        -> std::basic_format_parse_context<Char>::iterator {
        const auto spec_end{std::ranges::find(ctx.begin(), ctx.end(), static_cast<Char>('}'))};
        if (std::ranges::find(ctx.begin(), spec_end, static_cast<Char>('{')) != spec_end) {
            throw std::format_error(
                "dynamic width or precision is not supported for complex scalars"
            );
        }
        // [[fill]align][sign]...: replace or insert the sign option with '+' so that the imaginary
        // part always carries its sign.
        std::basic_string<Char> imag_spec(ctx.begin(), spec_end);
        const std::size_t pos{isAlign(imag_spec, 1) ? 2U : (isAlign(imag_spec, 0) ? 1U : 0U)};
        if (pos < imag_spec.size() && isSign(imag_spec[pos])) {
            imag_spec[pos] = static_cast<Char>('+');
        } else {
            imag_spec.insert(pos, 1, static_cast<Char>('+'));
        }
        std::basic_format_parse_context<Char> imag_ctx{std::basic_string_view<Char>{imag_spec}};
        m_imag.parse(imag_ctx);
        return m_real.parse(ctx);
    }

    template <typename FormatContext>
    auto format(const T& value, FormatContext& ctx) const -> FormatContext::iterator {
        auto out{ctx.out()};
        *out++ = static_cast<Char>('(');
        ctx.advance_to(out);
        ctx.advance_to(m_real.format(value.real(), ctx));
        out = m_imag.format(value.imag(), ctx);
        *out++ = static_cast<Char>('i');
        *out++ = static_cast<Char>(')');
        return out;
    }

  private:
    static constexpr auto isAlign(std::basic_string_view<Char> spec, std::size_t i) noexcept
        -> bool {
        return i < spec.size() &&
               (spec[i] == static_cast<Char>('<') || spec[i] == static_cast<Char>('>') ||
                spec[i] == static_cast<Char>('^'));
    }

    static constexpr auto isSign(Char c) noexcept -> bool {
        return c == static_cast<Char>('+') || c == static_cast<Char>('-') ||
               c == static_cast<Char>(' ');
    }

    std::formatter<typename T::value_type, Char> m_real;
    std::formatter<typename T::value_type, Char> m_imag;
};

} // namespace std
// NOLINTEND(cert-dcl58-cpp)
