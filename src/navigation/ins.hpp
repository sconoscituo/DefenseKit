#pragma once
#include "src/utils/constants.hpp"
#include <array>

namespace defensekit {
namespace navigation {

struct ImuData {
    double ax, ay, az;   // 가속도계 [m/s^2] (body frame)
    double gx, gy, gz;   // 자이로스코프 [rad/s] (body frame)
};

struct NavState {
    double x, y, z;      // 위치 [m] (NED 로컬 좌표계)
    double vx, vy, vz;   // 속도 [m/s]
    double roll, pitch, yaw; // 자세각 [rad]
};

// 관성항법시스템 (INS) 기초 구현
// Euler angle 기반 자세 추정 + 위치/속도 적분
class INS {
public:
    INS();

    // 초기 상태 설정
    void init(double x0, double y0, double z0,
              double roll0, double pitch0, double yaw0);

    // IMU 데이터로 상태 업데이트
    void update(const ImuData& imu, double dt);

    // 현재 항법 상태
    const NavState& state() const { return state_; }

    // 누적 경로 거리
    double odometry() const { return odometry_; }

    // 바이어스 교정 (정적 조건에서 초기화)
    void calibrate_bias(const ImuData& static_imu);

private:
    NavState state_{};
    double   odometry_{0.0};
    bool     initialized_{false};

    // 센서 바이어스
    double accel_bias_x_{0.0}, accel_bias_y_{0.0}, accel_bias_z_{0.0};
    double gyro_bias_x_{0.0},  gyro_bias_y_{0.0},  gyro_bias_z_{0.0};

    // body → NED 좌표 변환 (DCM: Direction Cosine Matrix)
    std::array<std::array<double, 3>, 3> dcm_body_to_ned() const;
};

} // namespace navigation
} // namespace defensekit
