/**
 * @file basjoo/math/integrate_mode.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Selector of the single step integration scheme.
 * @version 0.1
 * @date 2026-10-15
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <cstdint>

namespace basjoo::math {

enum class IntegrateMode : std::uint8_t {
    FORWARD_EULER,
    SYMPLECTIC_EULER,
    RUNGE_KUTTA
};

} // namespace basjoo::math
