#pragma once
#include <vector>
#include <cmath>

namespace defensekit {
namespace ballistics {

struct Vec3 {
    double x{0.0}, y{0.0}, z{0.0};

    Vec3 operator+(const Vec3& rhs) const { return {x+rhs.x, y+rhs.y, z+rhs.z}; }
    Vec3 operator-(const Vec3& rhs) const { return {x-rhs.x, y-rhs.y, z-rhs.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    double norm() const { return std::sqrt(x*x + y*y + z*z); }
};

struct ProjectileParams {
    double mass;             // 질량 [kg]
    double diameter;         // 직경 [m]
    double drag_coeff;       // 기준 항력 계수 (마하 의존성 적용 시 오버라이드)
    bool   use_mach_drag;    // 마하수 기반 항력 계수 사용 여부
    bool   use_coriolis;     // Coriolis 효과 포함 여부
    double latitude_deg;     // 발사 위도 [도] (Coriolis 계산용)
};

struct TrajectoryPoint {
    double time;    // [s]
    Vec3   pos;     // [m]
    Vec3   vel;     // [m/s]
    double speed;   // [m/s]
    double mach;    // 마하수
};

// 탄도 시뮬레이터
class ProjectileSimulator {
public:
    explicit ProjectileSimulator(const ProjectileParams& params);

    // 초기 조건으로 탄도 궤적 계산
    // elevation_deg: 앙각 [도], azimuth_deg: 방위각 [도], muzzle_velocity: 초속 [m/s]
    std::vector<TrajectoryPoint> simulate(
        double muzzle_velocity,
        double elevation_deg,
        double azimuth_deg = 0.0,
        double dt = 0.01
    ) const;

    // 최대 사거리 계산 (최적 앙각 탐색)
    double max_range(double muzzle_velocity) const;

    // 특정 목표 거리에 대한 필요 앙각 계산 (두 가지 해)
    bool solve_elevation(double muzzle_velocity, double range,
                         double& elevation_low, double& elevation_high) const;

private:
    ProjectileParams params_;
    double cross_section_;  // 단면적 [m^2]

    Vec3 compute_acceleration(const Vec3& pos, const Vec3& vel) const;
};

} // namespace ballistics
} // namespace defensekit
