#include <gtest/gtest.h>
#include "src/signal_processing/fft.hpp"
#include "src/utils/constants.hpp"
#include <cmath>
#include <vector>

using namespace defensekit::signal;
using namespace defensekit::constants;

TEST(FFTTest, DCComponent) {
    // 상수 신호 → DC 성분만 존재
    std::vector<double> signal(16, 1.0);
    auto spectrum = fft_real(signal);
    EXPECT_NEAR(std::abs(spectrum[0]), 16.0, 1e-6);
    for (int i = 1; i < 16; ++i)
        EXPECT_NEAR(std::abs(spectrum[i]), 0.0, 1e-6);
}

TEST(FFTTest, SingleFrequency) {
    // 순수 정현파 → 해당 주파수 빈에서 피크
    int    n           = 64;
    int    freq_bin    = 4;
    double sample_rate = 1000.0; // 1kHz
    double freq        = freq_bin * sample_rate / n; // 62.5Hz

    std::vector<double> signal(n);
    for (int i = 0; i < n; ++i)
        signal[i] = std::cos(2.0 * PI * freq * i / sample_rate);

    auto spectrum = fft_real(signal);

    // 해당 빈에서 최대값
    double max_mag = 0.0;
    int    max_bin = 0;
    for (int i = 1; i <= n / 2; ++i) {
        double mag = std::abs(spectrum[i]);
        if (mag > max_mag) { max_mag = mag; max_bin = i; }
    }
    EXPECT_EQ(max_bin, freq_bin);
}

TEST(FFTTest, InverseFFT) {
    // FFT → IFFT → 원신호 복원
    std::vector<Complex> data = {
        {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}, {4.0, 0.0},
        {4.0, 0.0}, {3.0, 0.0}, {2.0, 0.0}, {1.0, 0.0}
    };
    auto original = data;
    fft(data, false);
    fft(data, true);
    for (int i = 0; i < static_cast<int>(data.size()); ++i)
        EXPECT_NEAR(data[i].real(), original[i].real(), 1e-9);
}

TEST(FFTTest, PowerSpectrumDecibels) {
    std::vector<double> signal(32, 1.0);
    auto spectrum = fft_real(signal);
    auto psd      = power_spectrum_db(spectrum);
    EXPECT_EQ(psd.size(), spectrum.size());
    // DC 성분은 양수 dB여야 함
    EXPECT_GT(psd[0], 0.0);
}

TEST(FFTTest, FindPeakFrequency) {
    int    n           = 128;
    double sample_rate = 2000.0;
    double true_freq   = 250.0; // Hz

    std::vector<double> signal(n);
    for (int i = 0; i < n; ++i)
        signal[i] = std::sin(2.0 * PI * true_freq * i / sample_rate);

    auto [peak_freq, peak_power] = find_peak_frequency(signal, sample_rate);
    EXPECT_NEAR(peak_freq, true_freq, sample_rate / n + 1.0);
    EXPECT_GT(peak_power, 0.0);
}

TEST(FFTTest, NonPowerOfTwoThrows) {
    std::vector<Complex> data(3, {1.0, 0.0});
    EXPECT_THROW(fft(data), std::invalid_argument);
}
