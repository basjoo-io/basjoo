/**
 * @file utils.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-23
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <iterator>
#include <memory>
#include <memory_resource>
#include <ranges>
#include <vector>

#include "basjoo/math/concepts.hpp"

namespace basjoo::math {

inline constexpr double kEpsilon{1e-8};

struct periodic_tag final {};

[[using gnu: const, always_inline, leaf]]
inline constexpr auto pow(Arithmetic auto x, std::size_t n) noexcept -> decltype(x) {
    decltype(x) result = 1.0;
    for (; n > 0; --n) {
        result *= x;
    }
    return result;
}

[[using gnu: const, always_inline, leaf]]
inline constexpr auto inRange(
    Arithmetic auto value, Arithmetic auto start, Arithmetic auto end, double tol = 1E-8
) noexcept -> bool {
    return (value - start) * (value - end) < -tol;
}

template <GeneralArithmetic T, std::floating_point U = double>
[[using gnu: const, always_inline, leaf, hot]]
inline constexpr auto lerp(T start, T end, U ratio) noexcept -> T {
    return (1 - ratio) * start + ratio * end;
}

template <std::floating_point T>
[[using gnu: const, always_inline, leaf]]
inline auto wrapPeriodic(T t, T min_t, T max_t) noexcept -> T {
    const T period{max_t - min_t};
    T offset{std::fmod(t - min_t, period)};
    if (offset < static_cast<T>(0.0)) [[unlikely]] {
        offset += period;
    }
    return min_t + offset;
}

template <GeneralArithmetic T, std::weakly_incrementable OutputIt>
[[using gnu: always_inline, hot]]
inline auto linspace(
    T start, T end, std::size_t num, OutputIt result, bool endpoint = true
) noexcept -> OutputIt {
    if (num == 0) [[unlikely]] {
        return result;
    }
    if (num == 1) [[unlikely]] {
        *result++ = start;
        return result;
    }
    T step = endpoint ? (end - start) / static_cast<double>(num - 1)
                      : (end - start) / static_cast<double>(num);
    return std::ranges::generate_n(
        result, num,
        [t = start - step, h = step]() mutable constexpr noexcept -> T { return t += h; }
    );
}

template <GeneralArithmetic T, Allocatory Alloc = std::allocator<T>>
[[using gnu: pure, always_inline]]
inline auto linspace(
    T start, T end, std::size_t num, bool endpoint = true, const Alloc& alloc = {}
) noexcept -> std::vector<T, Alloc> {
    std::vector<T, Alloc> result(num, alloc);
    linspace(start, end, num, result.begin(), endpoint);
    return result;
}

template <std::input_iterator InputIt, std::sentinel_for<InputIt> SentinelIt>
[[using gnu: pure, always_inline]]
inline auto hasDuplicates(InputIt first, SentinelIt last, double tol = 1E-8) noexcept -> bool
    requires GeneralArithmetic<typename std::iterator_traits<InputIt>::value_type>
{
    using value_type = typename std::iterator_traits<InputIt>::value_type;
    return std::ranges::adjacent_find(
               first, last,
               [tol](const value_type& lhs, const value_type& rhs) constexpr noexcept -> bool {
                   return std::abs(lhs - rhs) < tol;
               }
           ) != last;
}

template <std::ranges::input_range InputRange>
[[using gnu: pure, always_inline]]
inline auto hasDuplicates(InputRange&& range, double tol = 1E-8) noexcept -> bool
    requires GeneralArithmetic<std::ranges::range_value_t<InputRange>>
{
    return hasDuplicates(std::ranges::begin(range), std::ranges::end(range), tol);
}

template <std::input_iterator InputIt, std::sentinel_for<InputIt> SentinelIt>
[[using gnu: pure, always_inline, hot]]
inline auto nearestUpperElement(
    InputIt first, SentinelIt last, typename std::iterator_traits<InputIt>::value_type element,
    double tol = 1E-8
) noexcept -> InputIt
    requires Arithmetic<typename std::iterator_traits<InputIt>::value_type>
{
    using value_type = typename std::iterator_traits<InputIt>::value_type;
    if (std::ranges::distance(first, last) < 2) [[unlikely]] {
        return first;
    }
    if (std::abs(element - *first) < tol) {
        return ++first;
    }
    if (std::abs(element - *(std::prev(last))) < tol) {
        return std::prev(last);
    }
    return std::ranges::upper_bound(
        first, last, element,
        [tol](const value_type& lhs, const value_type& rhs) constexpr noexcept -> bool {
            return lhs - rhs < -tol;
        }
    );
}

template <std::ranges::input_range InputRange>
[[using gnu: pure, always_inline, hot]]
inline auto nearestUpperElement(
    InputRange&& range, std::ranges::range_value_t<InputRange> element, double tol = 1E-8
) noexcept -> std::ranges::borrowed_iterator_t<InputRange>
    requires Arithmetic<std::ranges::range_value_t<InputRange>>
{
    return nearestUpperElement(std::ranges::begin(range), std::ranges::end(range), element, tol);
}

template <std::input_iterator InputIt, std::sentinel_for<InputIt> SentinelIt>
[[using gnu: pure, flatten, leaf, hot]]
inline auto nearestUpperElement(
    InputIt first, SentinelIt last, typename std::iterator_traits<InputIt>::value_type element,
    double tol = 1E-8
) noexcept -> InputIt
    requires VecArithmetic<typename std::iterator_traits<InputIt>::value_type>
{
    using value_type = typename std::iterator_traits<InputIt>::value_type;
    if (std::ranges::distance(first, last) < 2) [[unlikely]] {
        return first;
    }
    InputIt pos{first};
    double min_euclidean = std::numeric_limits<double>::max();
    for (auto it{first}; it != last; ++it) {
        const double euclidean = element.euclideanTo(*it);
        if (euclidean < min_euclidean) {
            pos = it;
            min_euclidean = euclidean;
        }
    }
    value_type diff, r;
    double inner_prod{0.0};
    if (pos == std::prev(last)) {
        diff = *std::prev(pos) - *pos;
        r = element - *pos;
        inner_prod = diff.dot(r);
        return inner_prod < -tol ? std::next(pos) : pos;
    }
    diff = *std::next(pos) - *pos;
    r = element - *pos;
    inner_prod = diff.dot(r);
    return inner_prod < -tol ? pos : std::next(pos);
}

template <std::ranges::input_range InputRange>
[[using gnu: pure, always_inline, hot]]
inline auto nearestUpperElement(
    InputRange&& range, std::ranges::range_value_t<InputRange> element, double tol = 1E-8
) noexcept -> std::ranges::iterator_t<InputRange>
    requires VecArithmetic<std::ranges::range_value_t<InputRange>>
{
    return nearestUpperElement(std::ranges::begin(range), std::ranges::end(range), element, tol);
}

} // namespace basjoo::math
