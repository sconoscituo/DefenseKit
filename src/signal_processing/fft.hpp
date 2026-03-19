#pragma once
#include <vector>
#include <complex>

namespace defensekit {
namespace signal {

using Complex = std::complex<double>;

// Cooley-Tukey FFT (재귀적 구현)
// 입력 크기는 2의 거듭제곱이어야 함
void fft(std::vector<Complex>& data, bool inverse = false);

// 실수 신호에 대한 FFT 래퍼
std::vector<Complex> fft_real(const std::vector<double>& signal);

// 주파수 빈 → 실제 주파수 변환 [Hz]
// bin: 빈 인덱스, n: FFT 크기, sample_rate: 샘플링 레이트 [Hz]
double bin_to_frequency(int bin, int n, double sample_rate);

// 파워 스펙트럼 밀도 계산 [dB]
std::vector<double> power_spectrum_db(const std::vector<Complex>& spectrum);

// 피크 주파수 탐색 (레이더 도플러 분석용)
// 반환: {주파수[Hz], 파워[dB]}
std::pair<double, double> find_peak_frequency(
    const std::vector<double>& signal,
    double sample_rate
);

} // namespace signal
} // namespace defensekit
