#include "src/radar/target_tracker.hpp"
#include "src/signal_processing/fft.hpp"
#include "src/utils/constants.hpp"
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <random>

using namespace defensekit::radar;
using namespace defensekit::signal;
using namespace defensekit::constants;

// 표적 시뮬레이션: 직선 등속 운동
struct SimTarget {
    double x, y;       // 현재 위치
    double vx, vy;     // 속도
    int    id;
};

RadarMeasurement simulate_radar_return(
    const SimTarget& target, double time,
    std::mt19937& rng, double noise_std = 20.0)
{
    std::normal_distribution<double> noise(0.0, noise_std);

    double mx = target.x + noise(rng);
    double my = target.y + noise(rng);

    double range   = std::sqrt(mx*mx + my*my);
    double azimuth = std::atan2(my, mx);

    return {time, range, azimuth, 0.0};
}

int main() {
    std::cout << "=== DefenseKit 레이더 추적 데모 ===\n\n";

    // FFT 도플러 분석 데모
    std::cout << "--- FFT 도플러 분석 ---\n";
    {
        double sample_rate = 10000.0; // 10kHz
        int    n           = 256;
        double doppler_freq = 1200.0; // 1.2kHz (표적 접근 속도 반영)

        std::vector<double> radar_signal(n);
        for (int i = 0; i < n; ++i) {
            radar_signal[i] = 1.5 * std::sin(2.0 * PI * doppler_freq * i / sample_rate)
                            + 0.3 * std::sin(2.0 * PI * 3000.0 * i / sample_rate); // 클러터
        }

        auto [peak_freq, peak_power] = find_peak_frequency(radar_signal, sample_rate);

        // 레이더 파장 (X-band: 3cm)
        double wavelength    = 0.03;
        double radial_speed  = peak_freq * wavelength / 2.0;

        std::cout << "  피크 주파수: " << std::fixed << std::setprecision(1)
                  << peak_freq << " Hz\n";
        std::cout << "  피크 파워:   " << std::setprecision(1)
                  << peak_power << " dB\n";
        std::cout << "  표적 접근 속도 (도플러): "
                  << std::setprecision(1) << radial_speed * 3.6 << " km/h\n\n";
    }

    // 다중 표적 추적 데모
    std::cout << "--- 다중 표적 추적 (칼만 필터) ---\n";
    {
        std::mt19937 rng(42);

        TargetTracker tracker(300.0, 3, 5);

        // 3개 표적 설정
        std::vector<SimTarget> targets = {
            {5000.0,  2000.0, -80.0,  10.0, 1},
            {3000.0, -1500.0, -60.0,  20.0, 2},
            {8000.0,  4000.0, -100.0, -5.0, 3},
        };

        std::cout << "초기 표적:\n";
        for (const auto& t : targets) {
            std::cout << "  표적 " << t.id
                      << ": 위치=(" << std::fixed << std::setprecision(0)
                      << t.x << ", " << t.y << ") m"
                      << "  속도=(" << t.vx << ", " << t.vy << ") m/s\n";
        }
        std::cout << "\n";

        std::cout << std::setw(6)  << "시간"
                  << std::setw(8)  << "트랙수"
                  << std::setw(8)  << "확정수"
                  << "  추정 위치\n";
        std::cout << std::string(60, '-') << "\n";

        double dt = 1.0; // 1초 간격 레이더 업데이트
        for (int frame = 0; frame < 15; ++frame) {
            double t = frame * dt;

            // 표적 이동
            for (auto& tgt : targets) {
                tgt.x += tgt.vx * dt;
                tgt.y += tgt.vy * dt;
            }

            // 레이더 측정값 생성
            std::vector<RadarMeasurement> measurements;
            for (const auto& tgt : targets) {
                if (tgt.x > 0 && tgt.x < 15000)  // 탐지 범위 내
                    measurements.push_back(simulate_radar_return(tgt, t, rng));
            }

            tracker.process_measurements(measurements, t);

            auto confirmed = tracker.get_confirmed_tracks();

            std::cout << std::setw(5) << std::fixed << std::setprecision(0) << t << "s"
                      << std::setw(8) << tracker.track_count()
                      << std::setw(8) << confirmed.size()
                      << "  ";

            for (const auto& tr : confirmed) {
                std::cout << "[T" << tr.id
                          << " (" << std::setprecision(0) << tr.x
                          << "," << tr.y << ")"
                          << " v=(" << std::setprecision(1) << tr.vx
                          << "," << tr.vy << ")] ";
            }
            std::cout << "\n";
        }
    }

    return 0;
}
