/**
 * @file test_fft.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-10-22
 *
 * @copyright Copyright (c) 2024 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/fft.hpp"

#include <complex>
#include <vector>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

using namespace std::literals::complex_literals;

TEST_CASE_TEMPLATE("FftTest", T, float, double, long double) {
    alignas(32) constexpr std::array<std::complex<T>, 4> data{1.0, 2.0, 3.0, 4.0};
    alignas(32) constexpr std::array<std::complex<T>, 4> expected_fft_result{
        std::complex<T>(10.0), std::complex<T>(-2.0, 2.0), std::complex<T>(-2.0),
        std::complex<T>(-2.0, -2.0)
    };
    std::pmr::vector<std::complex<T>> result(data.size());

    fft(data, result.begin());

    CHECK_EQ(result.size(), expected_fft_result.size());

    for (size_t i = 0; i < result.size(); ++i) {
        CHECK_EQ(result[i].real(), doctest::Approx(expected_fft_result[i].real()).epsilon(1E-8));
        CHECK_EQ(result[i].imag(), doctest::Approx(expected_fft_result[i].imag()).epsilon(1E-8));
    }
}

TEST_CASE_TEMPLATE("RfftTest", T, float, double, long double) {
    alignas(32) constexpr std::array<T, 4> data{1.0, 2.0, 3.0, 4.0};
    alignas(32) constexpr std::array<std::complex<T>, 3> expected_fft_result{
        std::complex<T>(10.0), std::complex<T>(-2.0, 2.0), std::complex<T>(-2.0)
    };
    std::pmr::vector<std::complex<T>> result(data.size() / 2 + 1);

    rfft(data, result.begin());

    CHECK_EQ(result.size(), expected_fft_result.size());

    for (size_t i = 0; i < result.size(); ++i) {
        CHECK_EQ(result[i].real(), doctest::Approx(expected_fft_result[i].real()).epsilon(1E-8));
        CHECK_EQ(result[i].imag(), doctest::Approx(expected_fft_result[i].imag()).epsilon(1E-8));
    }
}

TEST_CASE_TEMPLATE("IfftTest", T, float, double, long double) {
    alignas(32) constexpr std::array<std::complex<T>, 4> fft_data{
        std::complex<T>(10.0), std::complex<T>(-2.0, 2.0), std::complex<T>(-2.0),
        std::complex<T>(-2.0, -2.0)
    };
    alignas(32) constexpr std::array<std::complex<T>, 4> expected_ifft_result{1.0, 2.0, 3.0, 4.0};
    std::pmr::vector<std::complex<T>> result(fft_data.size());

    ifft(fft_data, result.begin());

    CHECK_EQ(result.size(), expected_ifft_result.size());

    for (size_t i = 0; i < result.size(); ++i) {
        CHECK_EQ(result[i].real(), doctest::Approx(expected_ifft_result[i].real()).epsilon(1E-8));
        CHECK_EQ(result[i].imag(), doctest::Approx(expected_ifft_result[i].imag()).epsilon(1E-8));
    }
}

TEST_CASE_TEMPLATE("IrfftTest", T, float, double, long double) {
    alignas(32) constexpr std::array<std::complex<T>, 3> rfft_data{
        std::complex<T>(10.0), std::complex<T>(-2.0, 2.0), std::complex<T>(-2.0)
    };
    alignas(32) constexpr std::array<T, 4> expected_irfft_result{1.0, 2.0, 3.0, 4.0};
    std::pmr::vector<T> result(rfft_data.size() * 2 - 2);

    irfft(rfft_data, result.begin());

    CHECK_EQ(result.size(), expected_irfft_result.size());

    for (size_t i = 0; i < result.size(); ++i) {
        CHECK_EQ(result[i], doctest::Approx(expected_irfft_result[i]).epsilon(1E-8));
    }
}

TEST_CASE_TEMPLATE("FftfreqTest", T, float, double, long double) {
    alignas(32) constexpr std::array<T, 8> expected_freq{0.0,  1.25,  2.5,  3.75,
                                                         -5.0, -3.75, -2.5, -1.25};
    std::pmr::vector<T> freq(8);

    fftfreq(freq, 0.1);

    CHECK_EQ(freq.size(), expected_freq.size());

    for (size_t i = 0; i < freq.size(); ++i) {
        CHECK_EQ(freq[i], doctest::Approx(expected_freq[i]).epsilon(1E-8));
    }
}

TEST_CASE_TEMPLATE("RfftfreqTest", T, float, double, long double) {
    alignas(32) constexpr std::array<T, 5> expected_freq{0.0, 1.25, 2.5, 3.75, 5.0};
    std::pmr::vector<T> freq(8 / 2 + 1);

    rfftfreq(freq, 0.1);

    CHECK_EQ(freq.size(), expected_freq.size());

    for (size_t i = 0; i < freq.size(); ++i) {
        CHECK_EQ(freq[i], doctest::Approx(expected_freq[i]).epsilon(1E-8));
    }
}

} // namespace basjoo::math
