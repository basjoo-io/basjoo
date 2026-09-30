/**
 * @file dualism.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-21
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <cstdint>

namespace basjoo::bicycle {

enum class Chirality : std::uint8_t {
    LEFT,
    RIGHT
};

enum class Actio : std::uint8_t {
    BLOCKING,
    PUSHING
};

} // namespace basjoo::bicycle
