/**
 * @file fft.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-10-22
 *
 * @copyright Copyright (c) 2024 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <complex>
#include <concepts>
#include <memory>
#include <ranges>
#include <stdexcept>
#include <utility>

#include "pocketfft_hdronly.h"

#include "basjoo/math/concepts.hpp"

namespace basjoo::math {

enum class FftNorm : std::uint8_t {
    BACKWARD = 0,
    ORTHO = 1,
    FORWARD = 2
};

template <
    std::contiguous_iterator InputIt, std::sentinel_for<InputIt> SentinelIt,
    std::weakly_incrementable OutputIt>
[[using gnu: always_inline]]
inline auto fft(
    InputIt first, SentinelIt last, OutputIt result, FftNorm norm = FftNorm::BACKWARD
) noexcept(!BASJOO_CHECK_PARAMS) -> std::ranges::in_out_result<InputIt, OutputIt>
    requires ::basjoo::math::ComplexArithmetic<
                 typename std::iterator_traits<InputIt>::value_type> &&
             std::same_as<
                 typename std::iterator_traits<OutputIt>::value_type,
                 typename std::iterator_traits<InputIt>::value_type>
{
    const std::size_t num_pnts = std::ranges::distance(first, last);
#if BASJOO_CHECK_PARAMS == 1
    if (num_pnts < 2) [[unlikely]] {
        throw std::invalid_argument("basjoo::math::fft(): input size must be greater than 1!");
    }
    if (norm > FftNorm::FORWARD) [[unlikely]] {
        throw std::invalid_argument("basjoo::math::fft(): invalid norm argument detected!");
    }
#endif

    using Complex = typename std::iterator_traits<InputIt>::value_type;
    using Scalar = typename Complex::value_type;

    Scalar fct = 1.0;
    switch (norm) {
    case FftNorm::BACKWARD:
        break;
    case FftNorm::ORTHO:
        fct /= std::sqrt(num_pnts);
        break;
    case FftNorm::FORWARD:
        fct /= num_pnts;
        break;
    [[unlikely]] default:
        std::unreachable();
    }

    pocketfft::c2c(
        {num_pnts}, {sizeof(Complex)}, {sizeof(Complex)}, {0}, pocketfft::FORWARD,
        std::to_address(first), std::to_address(result), fct
    );

    return {first + num_pnts, result};
}

template <std::ranges::contiguous_range InputRange, std::weakly_incrementable OutputIt>
[[using gnu: always_inline]]
inline auto fft(InputRange&& input, OutputIt result, FftNorm norm = FftNorm::BACKWARD) noexcept(
    !BASJOO_CHECK_PARAMS
) -> std::ranges::in_out_result<std::ranges::borrowed_iterator_t<InputRange>, OutputIt>
    requires ::basjoo::math::ComplexArithmetic<std::ranges::range_value_t<InputRange>> &&
             std::same_as<
                 typename std::iterator_traits<OutputIt>::value_type,
                 std::ranges::range_value_t<InputRange>>
{
    return fft(std::ranges::begin(input), std::ranges::end(input), result, norm);
}

template <
    std::contiguous_iterator InputIt, std::sentinel_for<InputIt> SentinelIt,
    std::contiguous_iterator OutputIt>
[[using gnu: always_inline]]
inline auto rfft(
    InputIt first, SentinelIt last, OutputIt result, FftNorm norm = FftNorm::BACKWARD
) noexcept(!BASJOO_CHECK_PARAMS) -> std::ranges::in_out_result<InputIt, OutputIt>
    requires std::floating_point<typename std::iterator_traits<InputIt>::value_type> &&
             std::same_as<
                 typename std::iterator_traits<OutputIt>::value_type,
                 std::complex<typename std::iterator_traits<InputIt>::value_type>>
{
    // output container should be longer than num_pnts / 2 + 1
    const std::size_t num_pnts = std::ranges::distance(first, last);
#if BASJOO_CHECK_PARAMS == 1
    if (num_pnts < 2) [[unlikely]] {
        throw std::invalid_argument("basjoo::math::rfft(): input size must be greater than 1!");
    }
    if (norm > FftNorm::FORWARD) [[unlikely]] {
        throw std::invalid_argument("basjoo::math::rfft(): invalid norm argument detected!");
    }
#endif

    using Scalar = typename std::iterator_traits<InputIt>::value_type;
    using Complex = std::complex<Scalar>;

    Scalar fct = 1.0;
    switch (norm) {
    case FftNorm::BACKWARD:
        break;
    case FftNorm::ORTHO:
        fct /= std::sqrt(num_pnts / 2 + 1);
        break;
    case FftNorm::FORWARD:
        fct /= num_pnts / 2 + 1;
        break;
    [[unlikely]] default:
        std::unreachable();
    }

    pocketfft::r2c(
        {num_pnts}, {sizeof(Scalar)}, {sizeof(Complex)}, {0}, pocketfft::FORWARD,
        std::to_address(first), std::to_address(result), fct
    );

    return {first + num_pnts, result};
}

template <std::ranges::contiguous_range InputRange, std::weakly_incrementable OutputIt>
[[using gnu: always_inline]]
inline auto rfft(InputRange&& input, OutputIt result, FftNorm norm = FftNorm::BACKWARD) noexcept(
    !BASJOO_CHECK_PARAMS
) -> std::ranges::in_out_result<std::ranges::borrowed_iterator_t<InputRange>, OutputIt>
    requires std::floating_point<std::ranges::range_value_t<InputRange>> &&
             std::same_as<
                 typename std::iterator_traits<OutputIt>::value_type,
                 std::complex<std::ranges::range_value_t<InputRange>>>
{
    return rfft(std::ranges::begin(input), std::ranges::end(input), result, norm);
}

template <
    std::contiguous_iterator InputIt, std::sentinel_for<InputIt> SentinelIt,
    std::weakly_incrementable OutputIt>
[[using gnu: always_inline]]
inline auto ifft(
    InputIt first, SentinelIt last, OutputIt result, FftNorm norm = FftNorm::BACKWARD
) noexcept(!BASJOO_CHECK_PARAMS) -> std::ranges::in_out_result<InputIt, OutputIt>
    requires ::basjoo::math::ComplexArithmetic<
                 typename std::iterator_traits<InputIt>::value_type> &&
             std::same_as<
                 typename std::iterator_traits<OutputIt>::value_type,
                 typename std::iterator_traits<InputIt>::value_type>
{
    const std::size_t num_pnts = std::ranges::distance(first, last);
#if BASJOO_CHECK_PARAMS == 1
    if (num_pnts < 2) [[unlikely]] {
        throw std::invalid_argument("basjoo::math::ifft(): input size must be greater than 1!");
    }
    if (norm > FftNorm::FORWARD) [[unlikely]] {
        throw std::invalid_argument("basjoo::math::ifft(): invalid norm argument detected!");
    }
#endif

    using Complex = typename std::iterator_traits<InputIt>::value_type;
    using Scalar = typename Complex::value_type;

    Scalar fct = 1.0;
    switch (norm) {
    case FftNorm::BACKWARD:
        fct /= num_pnts;
        break;
    case FftNorm::ORTHO:
        fct /= std::sqrt(num_pnts);
        break;
    case FftNorm::FORWARD:
        break;
    [[unlikely]] default:
        std::unreachable();
    }

    pocketfft::c2c(
        {num_pnts}, {sizeof(Complex)}, {sizeof(Complex)}, {0}, pocketfft::BACKWARD,
        std::to_address(first), std::to_address(result), fct
    );

    return {first + num_pnts, result};
}

template <std::ranges::contiguous_range InputRange, std::weakly_incrementable OutputIt>
[[using gnu: always_inline]]
inline auto ifft(InputRange&& input, OutputIt result, FftNorm norm = FftNorm::BACKWARD) noexcept(
    !BASJOO_CHECK_PARAMS
) -> std::ranges::in_out_result<std::ranges::borrowed_iterator_t<InputRange>, OutputIt>
    requires ::basjoo::math::ComplexArithmetic<std::ranges::range_value_t<InputRange>> &&
             std::same_as<
                 typename std::iterator_traits<OutputIt>::value_type,
                 std::ranges::range_value_t<InputRange>>
{
    return ifft(std::ranges::begin(input), std::ranges::end(input), result, norm);
}

template <
    std::contiguous_iterator InputIt, std::sentinel_for<InputIt> SentinelIt,
    std::weakly_incrementable OutputIt>
[[using gnu: always_inline]]
inline auto irfft(
    InputIt first, SentinelIt last, OutputIt result, FftNorm norm = FftNorm::BACKWARD
) noexcept(!BASJOO_CHECK_PARAMS) -> std::ranges::in_out_result<InputIt, OutputIt>
    requires ::basjoo::math::ComplexArithmetic<
                 typename std::iterator_traits<InputIt>::value_type> &&
             std::same_as<
                 std::complex<typename std::iterator_traits<OutputIt>::value_type>,
                 typename std::iterator_traits<InputIt>::value_type>
{
    // output container should be longer than num_pnts * 2 - 2
    const std::size_t num_pnts = std::ranges::distance(first, last);
#if BASJOO_CHECK_PARAMS == 1
    if (num_pnts < 2) [[unlikely]] {
        throw std::invalid_argument("basjoo::math::irfft(): input size must be greater than 1!");
    }
    if (norm > FftNorm::FORWARD) [[unlikely]] {
        throw std::invalid_argument("basjoo::math::irfft(): invalid norm argument detected!");
    }
#endif

    using Complex = typename std::iterator_traits<InputIt>::value_type;
    using Scalar = typename Complex::value_type;

    Scalar fct = 1.0;
    switch (norm) {
    case FftNorm::BACKWARD:
        fct /= num_pnts * 2 - 2;
        break;
    case FftNorm::ORTHO:
        fct /= std::sqrt(num_pnts * 2 - 2);
        break;
    case FftNorm::FORWARD:
        break;
    [[unlikely]] default:
        std::unreachable();
    }

    pocketfft::c2r(
        {num_pnts * 2 - 2}, {sizeof(Complex)}, {sizeof(Scalar)}, {0}, pocketfft::BACKWARD,
        std::to_address(first), std::to_address(result), fct
    );

    return {first + num_pnts, result};
}

template <std::ranges::contiguous_range InputRange, std::weakly_incrementable OutputIt>
[[using gnu: always_inline]]
inline auto irfft(InputRange&& input, OutputIt result, FftNorm norm = FftNorm::BACKWARD) noexcept(
    !BASJOO_CHECK_PARAMS
) -> std::ranges::in_out_result<std::ranges::borrowed_iterator_t<InputRange>, OutputIt>
    requires ::basjoo::math::ComplexArithmetic<std::ranges::range_value_t<InputRange>> &&
             std::same_as<
                 std::complex<typename std::iterator_traits<OutputIt>::value_type>,
                 std::ranges::range_value_t<InputRange>>
{
    return irfft(std::ranges::begin(input), std::ranges::end(input), result, norm);
}

template <std::contiguous_iterator OutputIt, std::sentinel_for<OutputIt> SentinelIt>
[[using gnu: always_inline]]
inline auto fftfreq(
    OutputIt first, SentinelIt last,
    typename std::iterator_traits<OutputIt>::value_type spacing = 1.0
) noexcept -> OutputIt
    requires std::floating_point<typename std::iterator_traits<OutputIt>::value_type>
{
    using Scalar = typename std::iterator_traits<OutputIt>::value_type;
    const std::ptrdiff_t n{std::ranges::distance(first, last)};
    const Scalar val = 1.0 / (n * spacing);
    const int mid = (n - 1) / 2 + 1;

    std::ranges::generate(
        first, first + mid,
        [t = -val, h = val]() mutable constexpr noexcept -> Scalar { return t += h; }
    );
    std::ranges::generate(
        first + mid, last,
        [t = -(n / 2 + 1) * val, h = val]() mutable constexpr noexcept -> Scalar { return t += h; }
    );

    return first + n;
}

template <std::ranges::contiguous_range OutputRange>
[[using gnu: always_inline]]
inline auto fftfreq(
    OutputRange&& output, typename std::ranges::range_value_t<OutputRange> spacing = 1.0
) noexcept -> std::ranges::borrowed_iterator_t<OutputRange>
    requires std::floating_point<typename std::ranges::range_value_t<OutputRange>>
{
    return fftfreq(std::ranges::begin(output), std::ranges::end(output), spacing);
}

template <std::contiguous_iterator OutputIt, std::sentinel_for<OutputIt> SentinelIt>
[[using gnu: always_inline]]
inline auto rfftfreq(
    OutputIt first, SentinelIt last,
    typename std::iterator_traits<OutputIt>::value_type spacing = 1.0
) noexcept -> OutputIt
    requires std::floating_point<typename std::iterator_traits<OutputIt>::value_type>
{

    using Scalar = typename std::iterator_traits<OutputIt>::value_type;
    const std::ptrdiff_t n{std::ranges::distance(first, last)};
    const Scalar val = 1.0 / ((n * 2 - 2) * spacing);

    std::ranges::generate(first, last, [t = -val, h = val]() mutable constexpr noexcept -> Scalar {
        return t += h;
    });

    return first + n;
}

template <std::ranges::contiguous_range OutputRange>
[[using gnu: always_inline]]
inline auto rfftfreq(
    OutputRange&& output, typename std::ranges::range_value_t<OutputRange> spacing = 1.0
) noexcept -> std::ranges::borrowed_iterator_t<OutputRange>
    requires std::floating_point<typename std::ranges::range_value_t<OutputRange>>
{
    return rfftfreq(std::ranges::begin(output), std::ranges::end(output), spacing);
}

} // namespace basjoo::math
