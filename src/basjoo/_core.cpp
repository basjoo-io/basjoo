/**
 * @file _core.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief nanobind module entry point for basjoo._core.
 * @version 0.1
 * @date 2026-08-11
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#include <nanobind/nanobind.h>

NB_MODULE(_core, m) { m.doc() = "Low-level C++ bindings for basjoo."; }
