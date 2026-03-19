#include "src/navigation/ins.hpp"
#include <cmath>
#include <stdexcept>

namespace defensekit {
namespace navigation {

using namespace constants;

INS::INS() {
    state_ = {0, 0, 0, 0, 0, 0, 0, 0, 0};
}

void INS::init(double x0, double y0, double z0,
               double roll0, double pitch0, double yaw0) {
    state_.x     = x0;   state_.y     = y0;   state_.z     = z0;
    state_.vx    = 0.0;  state_.vy    = 0.0;  state_.vz    = 0.0;
    state_.roll  = roll0; state_.pitch = pitch0; state_.yaw  = yaw0;
    odometry_    = 0.0;
    initialized_ = true;
}

std::array<std::array<double, 3>, 3> INS::dcm_body_to_ned() const {
    double cr = std::cos(state_.roll),  sr = std::sin(state_.roll);
    double cp = std::cos(state_.pitch), sp = std::sin(state_.pitch);
    double cy = std::cos(state_.yaw),   sy = std::sin(state_.yaw);

    // ZYX Euler 순서 (yaw → pitch → roll)
    return {{
        {cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr},
        {sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr},
        {-sp,    cp*sr,              cp*cr           }
    }};
}

void INS::update(const ImuData& imu, double dt) {
    if (!initialized_)
        throw std::runtime_error("INS: init()를 먼저 호출하세요");

    // 바이어스 보정
    double ax = imu.ax - accel_bias_x_;
    double ay = imu.ay - accel_bias_y_;
    double az = imu.az - accel_bias_z_;
    double gx = imu.gx - gyro_bias_x_;
    double gy = imu.gy - gyro_bias_y_;
    double gz = imu.gz - gyro_bias_z_;

    // 자세 업데이트 (Euler angle 적분)
    // 자이로 angular rate → Euler rate 변환
    double cr = std::cos(state_.roll),  sr = std::sin(state_.roll);
    double cp = std::cos(state_.pitch), tp = std::tan(state_.pitch);

    double roll_rate  = gx + (gy*sr + gz*cr) * tp;
    double pitch_rate = gy*cr - gz*sr;
    double yaw_rate   = (gy*sr + gz*cr) / cp;

    state_.roll  += roll_rate  * dt;
    state_.pitch += pitch_rate * dt;
    state_.yaw   += yaw_rate   * dt;

    // 자세각 정규화 [-PI, PI]
    auto normalize_angle = [](double a) {
        while (a >  PI) a -= 2.0 * PI;
        while (a < -PI) a += 2.0 * PI;
        return a;
    };
    state_.roll  = normalize_angle(state_.roll);
    state_.pitch = normalize_angle(state_.pitch);
    state_.yaw   = normalize_angle(state_.yaw);

    // body frame 가속도 → NED frame 변환
    auto dcm = dcm_body_to_ned();
    double ned_ax = dcm[0][0]*ax + dcm[0][1]*ay + dcm[0][2]*az;
    double ned_ay = dcm[1][0]*ax + dcm[1][1]*ay + dcm[1][2]*az;
    double ned_az = dcm[2][0]*ax + dcm[2][1]*ay + dcm[2][2]*az;

    // 중력 보상 (NED z 방향이 아래)
    ned_az += GRAVITY;

    // 속도 적분
    double prev_vx = state_.vx, prev_vy = state_.vy, prev_vz = state_.vz;
    state_.vx += ned_ax * dt;
    state_.vy += ned_ay * dt;
    state_.vz += ned_az * dt;

    // 위치 적분 (사다리꼴법)
    state_.x += 0.5 * (prev_vx + state_.vx) * dt;
    state_.y += 0.5 * (prev_vy + state_.vy) * dt;
    state_.z += 0.5 * (prev_vz + state_.vz) * dt;

    // 누적 거리
    double dv = std::sqrt(state_.vx*state_.vx + state_.vy*state_.vy + state_.vz*state_.vz);
    odometry_ += dv * dt;
}

void INS::calibrate_bias(const ImuData& static_imu) {
    // 정적 상태 가정: 가속도 = 중력만, 자이로 = 0
    accel_bias_x_ = static_imu.ax;
    accel_bias_y_ = static_imu.ay;
    accel_bias_z_ = static_imu.az + GRAVITY; // z 방향 중력 보상
    gyro_bias_x_  = static_imu.gx;
    gyro_bias_y_  = static_imu.gy;
    gyro_bias_z_  = static_imu.gz;
}

} // namespace navigation
} // namespace defensekit
