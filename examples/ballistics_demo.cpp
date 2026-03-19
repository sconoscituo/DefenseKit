#include "src/ballistics/projectile.hpp"
#include "src/ballistics/atmosphere.hpp"
#include "src/utils/constants.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>

using namespace defensekit::ballistics;
using namespace defensekit::constants;

int main() {
    std::cout << "=== DefenseKit 탄도 시뮬레이션 데모 ===\n\n";

    // 155mm 포탄 파라미터 (근사치)
    ProjectileParams params{
        .mass          = 43.0,    // 43kg
        .diameter      = 0.155,   // 155mm
        .drag_coeff    = 0.30,
        .use_mach_drag = true,
        .use_coriolis  = true,
        .latitude_deg  = 37.5,    // 한반도 중부
    };

    ProjectileSimulator sim(params);

    double muzzle_velocity = 827.0; // m/s (K9 자주포 초속)
    double elevation_deg   = 45.0;

    std::cout << "발사 조건:\n";
    std::cout << "  초속: " << muzzle_velocity << " m/s\n";
    std::cout << "  앙각: " << elevation_deg << "도\n";
    std::cout << "  질량: " << params.mass << " kg\n";
    std::cout << "  직경: " << params.diameter * 1000 << " mm\n\n";

    auto traj = sim.simulate(muzzle_velocity, elevation_deg, 0.0, 0.1);

    // 궤적 출력 (10초 간격)
    std::cout << std::setw(8) << "시간(s)"
              << std::setw(12) << "수평거리(m)"
              << std::setw(12) << "고도(m)"
              << std::setw(10) << "속도(m/s)"
              << std::setw(8)  << "마하수"
              << "\n";
    std::cout << std::string(50, '-') << "\n";

    double prev_time = -1.0;
    for (const auto& pt : traj) {
        if (pt.time - prev_time >= 10.0) {
            std::cout << std::fixed << std::setprecision(1)
                      << std::setw(8)  << pt.time
                      << std::setw(12) << pt.pos.x
                      << std::setw(12) << pt.pos.y
                      << std::setw(10) << pt.speed
                      << std::setw(8)  << std::setprecision(2) << pt.mach
                      << "\n";
            prev_time = pt.time;
        }
    }

    if (!traj.empty()) {
        const auto& last = traj.back();
        double range = std::sqrt(last.pos.x*last.pos.x + last.pos.z*last.pos.z);
        std::cout << "\n탄도 결과:\n";
        std::cout << "  비행 시간: " << std::fixed << std::setprecision(1)
                  << last.time << " 초\n";
        std::cout << "  최종 사거리: " << std::setprecision(0)
                  << range << " m ("
                  << range / 1000.0 << " km)\n";
    }

    // 최적 앙각 탐색
    std::cout << "\n--- 앙각별 사거리 ---\n";
    for (int deg : {20, 30, 40, 45, 50, 60, 70}) {
        auto t = sim.simulate(muzzle_velocity, static_cast<double>(deg), 0.0, 0.1);
        if (!t.empty()) {
            double r = t.back().pos.x;
            std::cout << "  " << deg << "도: "
                      << std::fixed << std::setprecision(0)
                      << r << " m\n";
        }
    }

    // 역탄도 계산 (목표 거리에 대한 앙각)
    double target_range = 20000.0; // 20km
    double elev_low = 0.0, elev_high = 0.0;
    if (sim.solve_elevation(muzzle_velocity, target_range, elev_low, elev_high)) {
        std::cout << "\n역탄도 계산 (목표 " << target_range / 1000.0 << "km):\n";
        std::cout << "  저각 사격: "
                  << std::fixed << std::setprecision(2) << elev_low << "도\n";
        std::cout << "  고각 사격: "
                  << std::setprecision(2) << elev_high << "도\n";
    }

    return 0;
}
