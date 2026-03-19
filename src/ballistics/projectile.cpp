#include "src/ballistics/projectile.hpp"
#include "src/ballistics/atmosphere.hpp"
#include "src/utils/constants.hpp"
#include <cmath>
#include <algorithm>

namespace defensekit {
namespace ballistics {

using namespace constants;

ProjectileSimulator::ProjectileSimulator(const ProjectileParams& params)
    : params_(params)
    , cross_section_(PI * (params.diameter / 2.0) * (params.diameter / 2.0))
{}

Vec3 ProjectileSimulator::compute_acceleration(const Vec3& pos, const Vec3& vel) const {
    double altitude = pos.y;
    double speed = vel.norm();

    // 중력
    Vec3 accel{0.0, -GRAVITY, 0.0};

    // 공기저항 (항력)
    if (speed > 1e-6) {
        auto atm = get_atmosphere(altitude);
        double mach = speed / atm.speed_of_sound;
        double cd = params_.use_mach_drag ? drag_coefficient(mach) : params_.drag_coeff;

        // F_drag = 0.5 * rho * v^2 * Cd * A
        double drag_force = 0.5 * atm.density * speed * speed * cd * cross_section_;
        double drag_accel = drag_force / params_.mass;

        // 속도 반대 방향
        accel.x -= drag_accel * vel.x / speed;
        accel.y -= drag_accel * vel.y / speed;
        accel.z -= drag_accel * vel.z / speed;
    }

    // Coriolis 효과: a_cor = -2 * (omega x v)
    // omega = [0, omega*cos(lat), omega*sin(lat)] (북쪽 기준)
    if (params_.use_coriolis && speed > 1e-6) {
        double lat_rad = params_.latitude_deg * DEG_TO_RAD;
        double omega_y = EARTH_OMEGA * std::cos(lat_rad);
        double omega_z = EARTH_OMEGA * std::sin(lat_rad);

        // omega x v (x 성분은 0으로 단순화)
        double cor_x =  2.0 * (omega_y * vel.z - omega_z * vel.y);
        double cor_y = -2.0 * omega_y * vel.x; // 단순화된 2D 근사
        double cor_z =  2.0 * omega_z * vel.x;

        accel.x -= cor_x;
        accel.y -= cor_y;
        accel.z -= cor_z;
    }

    return accel;
}

std::vector<TrajectoryPoint> ProjectileSimulator::simulate(
    double muzzle_velocity,
    double elevation_deg,
    double azimuth_deg,
    double dt) const
{
    std::vector<TrajectoryPoint> trajectory;
    trajectory.reserve(10000);

    double elev_rad = elevation_deg * DEG_TO_RAD;
    double azim_rad = azimuth_deg   * DEG_TO_RAD;

    Vec3 pos{0.0, 0.0, 0.0};
    Vec3 vel{
        muzzle_velocity * std::cos(elev_rad) * std::cos(azim_rad),
        muzzle_velocity * std::sin(elev_rad),
        muzzle_velocity * std::cos(elev_rad) * std::sin(azim_rad)
    };

    double t = 0.0;

    while (pos.y >= 0.0 || t < dt * 2) {
        auto atm = get_atmosphere(pos.y);
        double speed = vel.norm();
        double mach  = speed / atm.speed_of_sound;

        trajectory.push_back({t, pos, vel, speed, mach});

        // Runge-Kutta 4차 적분
        Vec3 k1v = compute_acceleration(pos, vel);
        Vec3 k1p = vel;

        Vec3 p2 = pos + k1p * (dt / 2.0);
        Vec3 v2 = vel + k1v * (dt / 2.0);
        Vec3 k2v = compute_acceleration(p2, v2);
        Vec3 k2p = v2;

        Vec3 p3 = pos + k2p * (dt / 2.0);
        Vec3 v3 = vel + k2v * (dt / 2.0);
        Vec3 k3v = compute_acceleration(p3, v3);
        Vec3 k3p = v3;

        Vec3 p4 = pos + k3p * dt;
        Vec3 v4 = vel + k3v * dt;
        Vec3 k4v = compute_acceleration(p4, v4);
        Vec3 k4p = v4;

        pos = pos + (k1p + k2p * 2.0 + k3p * 2.0 + k4p) * (dt / 6.0);
        vel = vel + (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (dt / 6.0);
        t  += dt;

        if (pos.y < 0.0) break;
        if (t > 3600.0) break; // 안전 한계
    }

    return trajectory;
}

double ProjectileSimulator::max_range(double muzzle_velocity) const {
    double best_range = 0.0;
    for (int deg = 1; deg <= 89; ++deg) {
        auto traj = simulate(muzzle_velocity, static_cast<double>(deg));
        if (!traj.empty()) {
            double range = std::sqrt(traj.back().pos.x * traj.back().pos.x +
                                     traj.back().pos.z * traj.back().pos.z);
            best_range = std::max(best_range, range);
        }
    }
    return best_range;
}

bool ProjectileSimulator::solve_elevation(
    double muzzle_velocity, double target_range,
    double& elevation_low, double& elevation_high) const
{
    // 이분법으로 두 해 탐색 (저각/고각)
    auto get_range = [&](double elev) {
        auto traj = simulate(muzzle_velocity, elev);
        if (traj.empty()) return 0.0;
        return std::sqrt(traj.back().pos.x * traj.back().pos.x +
                         traj.back().pos.z * traj.back().pos.z);
    };

    // 최대 사거리 각도 탐색 (~45도 근방)
    double best_elev = 45.0;
    double best_range = 0.0;
    for (int d = 30; d <= 60; ++d) {
        double r = get_range(static_cast<double>(d));
        if (r > best_range) { best_range = r; best_elev = d; }
    }

    if (target_range > best_range) return false;

    // 저각 탐색 [1, best_elev]
    double lo = 1.0, hi = best_elev;
    for (int i = 0; i < 50; ++i) {
        double mid = (lo + hi) / 2.0;
        if (get_range(mid) < target_range) lo = mid; else hi = mid;
    }
    elevation_low = (lo + hi) / 2.0;

    // 고각 탐색 [best_elev, 89]
    lo = best_elev; hi = 89.0;
    for (int i = 0; i < 50; ++i) {
        double mid = (lo + hi) / 2.0;
        if (get_range(mid) > target_range) lo = mid; else hi = mid;
    }
    elevation_high = (lo + hi) / 2.0;
    return true;
}

} // namespace ballistics
} // namespace defensekit
