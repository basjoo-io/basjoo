/**
 * @file in_in_in_out_result.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-07-03
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>

namespace basjoo::common {

template <typename InputIt1, typename InputIt2, typename InputIt3, typename OutputIt>
struct InInInOutResult final {
    [[no_unique_address]] InputIt1 in1;
    [[no_unique_address]] InputIt2 in2;
    [[no_unique_address]] InputIt3 in3;
    [[no_unique_address]] OutputIt out;

    // optional conversion operators
    template <
        typename OtherInputIt1, typename OtherInputIt2, typename OtherInputIt3,
        typename OtherOutputIt>
        requires std::convertible_to<InputIt1, OtherInputIt1> &&
                 std::convertible_to<InputIt2, OtherInputIt2> &&
                 std::convertible_to<InputIt3, OtherInputIt3> &&
                 std::convertible_to<OutputIt, OtherOutputIt>
    [[using gnu: always_inline]]
    constexpr operator InInInOutResult<
        OtherInputIt1, OtherInputIt2, OtherInputIt3, OtherOutputIt>() const& noexcept {
        return {in1, in2, in3, out};
    }

    template <
        typename OtherInputIt1, typename OtherInputIt2, typename OtherInputIt3,
        typename OtherOutputIt>
        requires std::convertible_to<InputIt1, OtherInputIt1> &&
                 std::convertible_to<InputIt2, OtherInputIt2> &&
                 std::convertible_to<InputIt3, OtherInputIt3> &&
                 std::convertible_to<OutputIt, OtherOutputIt>
    [[using gnu: always_inline]]
    constexpr operator InInInOutResult<
        OtherInputIt1, OtherInputIt2, OtherInputIt3, OtherOutputIt>() && noexcept {
        return {std::move(in1), std::move(in2), std::move(in3), std::move(out)};
    }
};

} // namespace basjoo::common
