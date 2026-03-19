#include "src/signal_processing/fft.hpp"
#include "src/utils/constants.hpp"
#include <cmath>
#include <stdexcept>
#include <algorithm>

namespace defensekit {
namespace signal {

using namespace constants;

// 비트 역순 치환 (Cooley-Tukey 전처리)
static void bit_reverse_permutation(std::vector<Complex>& data) {
    int n = static_cast<int>(data.size());
    int j = 0;
    for (int i = 1; i < n; ++i) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(data[i], data[j]);
    }
}

// Cooley-Tukey FFT (반복적 구현 - 캐시 효율적)
void fft(std::vector<Complex>& data, bool inverse) {
    int n = static_cast<int>(data.size());
    if (n <= 1) return;

    // 2의 거듭제곱 검증
    if ((n & (n - 1)) != 0)
        throw std::invalid_argument("FFT 크기는 2의 거듭제곱이어야 합니다");

    bit_reverse_permutation(data);

    // 버터플라이 연산
    for (int len = 2; len <= n; len <<= 1) {
        double angle = 2.0 * PI / len * (inverse ? 1.0 : -1.0);
        Complex w_len(std::cos(angle), std::sin(angle));

        for (int i = 0; i < n; i += len) {
            Complex w(1.0, 0.0);
            for (int j = 0; j < len / 2; ++j) {
                Complex u = data[i + j];
                Complex v = data[i + j + len / 2] * w;
                data[i + j]           = u + v;
                data[i + j + len / 2] = u - v;
                w *= w_len;
            }
        }
    }

    // 역변환 정규화
    if (inverse) {
        for (auto& val : data)
            val /= static_cast<double>(n);
    }
}

std::vector<Complex> fft_real(const std::vector<double>& signal) {
    // 2의 거듭제곱으로 패딩
    int n = 1;
    while (n < static_cast<int>(signal.size()))
        n <<= 1;

    std::vector<Complex> data(n, Complex(0.0, 0.0));
    for (int i = 0; i < static_cast<int>(signal.size()); ++i)
        data[i] = Complex(signal[i], 0.0);

    fft(data);
    return data;
}

double bin_to_frequency(int bin, int n, double sample_rate) {
    if (bin <= n / 2)
        return static_cast<double>(bin) * sample_rate / n;
    else
        return static_cast<double>(bin - n) * sample_rate / n;
}

std::vector<double> power_spectrum_db(const std::vector<Complex>& spectrum) {
    std::vector<double> psd(spectrum.size());
    for (int i = 0; i < static_cast<int>(spectrum.size()); ++i) {
        double magnitude = std::abs(spectrum[i]);
        // 0 방지 위해 1e-12 floor
        psd[i] = 20.0 * std::log10(std::max(magnitude, 1e-12));
    }
    return psd;
}

std::pair<double, double> find_peak_frequency(
    const std::vector<double>& signal,
    double sample_rate)
{
    auto spectrum = fft_real(signal);
    int n = static_cast<int>(spectrum.size());

    // DC 성분 제외, 나이퀴스트 이하 구간만 탐색
    int peak_bin = 1;
    double peak_power = -1e18;
    for (int i = 1; i <= n / 2; ++i) {
        double power = 20.0 * std::log10(std::max(std::abs(spectrum[i]), 1e-12));
        if (power > peak_power) {
            peak_power = power;
            peak_bin   = i;
        }
    }

    double peak_freq = bin_to_frequency(peak_bin, n, sample_rate);
    return {peak_freq, peak_power};
}

} // namespace signal
} // namespace defensekit
