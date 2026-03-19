#pragma once
#include "src/utils/matrix.hpp"

namespace defensekit {
namespace signal {

// 선형 칼만 필터 (4차원 상태: [x, y, vx, vy])
// 레이더 표적 추적에 사용 - 위치 측정값으로 위치+속도 추정
class KalmanFilter {
public:
    // 상태 벡터: [x, y, vx, vy]
    // 측정 벡터: [x, y]
    KalmanFilter();

    // 초기 상태 설정
    void init(double x0, double y0, double vx0, double vy0);

    // 예측 단계 (시간 dt만큼 전파)
    void predict(double dt);

    // 업데이트 단계 (레이더 측정값 반영)
    void update(double meas_x, double meas_y);

    // 현재 추정 상태 접근자
    double x()  const { return state_(0, 0); }
    double y()  const { return state_(1, 0); }
    double vx() const { return state_(2, 0); }
    double vy() const { return state_(3, 0); }

    // 공분산 행렬 (추정 불확실성)
    const Matrix<4, 4>& covariance() const { return P_; }

    // 노이즈 파라미터 설정
    void set_process_noise(double q);        // 프로세스 노이즈 분산
    void set_measurement_noise(double r_x, double r_y); // 측정 노이즈 분산

private:
    Matrix<4, 1> state_;   // 상태 벡터 [x, y, vx, vy]
    Matrix<4, 4> P_;       // 오차 공분산
    Matrix<4, 4> Q_;       // 프로세스 노이즈 공분산
    Matrix<2, 2> R_;       // 측정 노이즈 공분산
    Matrix<2, 4> H_;       // 관측 행렬 (위치만 측정)

    bool initialized_{false};
};

} // namespace signal
} // namespace defensekit
