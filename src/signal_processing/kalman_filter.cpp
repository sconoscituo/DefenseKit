#include "src/signal_processing/kalman_filter.hpp"
#include <stdexcept>

namespace defensekit {
namespace signal {

KalmanFilter::KalmanFilter() {
    // 관측 행렬: 상태 [x, y, vx, vy] 중 [x, y]만 측정
    H_ = Matrix<2, 4>{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0}
    };

    // 기본 측정 노이즈 공분산 (레이더 오차 ~10m)
    R_(0, 0) = 100.0;  // x 분산 [m^2]
    R_(1, 1) = 100.0;  // y 분산 [m^2]

    // 기본 프로세스 노이즈 (가속도 불확실성 ~1 m/s^2)
    double q = 1.0;
    Q_(0, 0) = q; Q_(1, 1) = q;
    Q_(2, 2) = q * 4.0; Q_(3, 3) = q * 4.0;

    // 초기 오차 공분산 (큰 불확실성)
    P_ = Matrix<4, 4>::identity() * 1000.0;
}

void KalmanFilter::init(double x0, double y0, double vx0, double vy0) {
    state_(0, 0) = x0;
    state_(1, 0) = y0;
    state_(2, 0) = vx0;
    state_(3, 0) = vy0;
    P_ = Matrix<4, 4>::identity() * 1000.0;
    initialized_ = true;
}

void KalmanFilter::predict(double dt) {
    if (!initialized_)
        throw std::runtime_error("KalmanFilter: init()를 먼저 호출하세요");

    // 상태 전이 행렬 F (등속 운동 모델)
    // [1  0  dt  0 ]
    // [0  1  0   dt]
    // [0  0  1   0 ]
    // [0  0  0   1 ]
    Matrix<4, 4> F = Matrix<4, 4>::identity();
    F(0, 2) = dt;
    F(1, 3) = dt;

    // 상태 예측: x_pred = F * x
    state_ = F * state_;

    // 공분산 예측: P_pred = F * P * F^T + Q
    P_ = F * P_ * F.transpose() + Q_;
}

void KalmanFilter::update(double meas_x, double meas_y) {
    if (!initialized_)
        throw std::runtime_error("KalmanFilter: init()를 먼저 호출하세요");

    // 측정 벡터
    Matrix<2, 1> z;
    z(0, 0) = meas_x;
    z(1, 0) = meas_y;

    // 혁신(Innovation): y = z - H * x
    Matrix<2, 1> innovation = z - H_ * state_;

    // 혁신 공분산: S = H * P * H^T + R
    Matrix<2, 2> S = H_ * P_ * H_.transpose() + R_;

    // 칼만 게인: K = P * H^T * S^-1
    Matrix<2, 2> S_inv = inverse2x2(S);
    Matrix<4, 2> K = P_ * H_.transpose() * S_inv;

    // 상태 업데이트: x = x + K * y
    state_ = state_ + K * innovation;

    // 공분산 업데이트: P = (I - K * H) * P
    Matrix<4, 4> I = Matrix<4, 4>::identity();
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::set_process_noise(double q) {
    Q_ = Matrix<4, 4>{};
    Q_(0, 0) = q;       Q_(1, 1) = q;
    Q_(2, 2) = q * 4.0; Q_(3, 3) = q * 4.0;
}

void KalmanFilter::set_measurement_noise(double r_x, double r_y) {
    R_(0, 0) = r_x * r_x;
    R_(1, 1) = r_y * r_y;
}

} // namespace signal
} // namespace defensekit
