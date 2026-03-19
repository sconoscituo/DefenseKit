#include <gtest/gtest.h>
#include "src/signal_processing/kalman_filter.hpp"
#include <cmath>

using namespace defensekit::signal;

TEST(KalmanFilterTest, InitAndPredict) {
    KalmanFilter kf;
    kf.init(0.0, 0.0, 100.0, 50.0);

    kf.predict(1.0);

    // 등속 운동: 1초 후 x=100, y=50
    EXPECT_NEAR(kf.x(), 100.0, 1e-6);
    EXPECT_NEAR(kf.y(),  50.0, 1e-6);
    EXPECT_NEAR(kf.vx(), 100.0, 1e-6);
    EXPECT_NEAR(kf.vy(),  50.0, 1e-6);
}

TEST(KalmanFilterTest, UpdateReducesUncertainty) {
    KalmanFilter kf;
    kf.init(0.0, 0.0, 10.0, 0.0);

    double initial_var = kf.covariance()(0, 0);

    for (int i = 0; i < 10; ++i) {
        kf.predict(0.1);
        kf.update(static_cast<double>(i) * 1.0 + 0.05,
                  0.0);
    }

    double final_var = kf.covariance()(0, 0);
    EXPECT_LT(final_var, initial_var);
}

TEST(KalmanFilterTest, ConvergesOnConstantVelocity) {
    KalmanFilter kf;
    kf.set_measurement_noise(5.0, 5.0);
    kf.init(0.0, 0.0, 50.0, 0.0);

    double true_vx = 50.0;
    double dt      = 0.1;

    for (int i = 1; i <= 50; ++i) {
        kf.predict(dt);
        // 측정값에 작은 노이즈 추가 (결정론적 테스트)
        double noise = (i % 2 == 0) ? 2.0 : -2.0;
        kf.update(true_vx * i * dt + noise, noise * 0.5);
    }

    // 속도 추정이 실제 속도에 수렴해야 함
    EXPECT_NEAR(kf.vx(), true_vx, 5.0);
}

TEST(KalmanFilterTest, ThrowsWithoutInit) {
    KalmanFilter kf;
    EXPECT_THROW(kf.predict(0.1), std::runtime_error);
    EXPECT_THROW(kf.update(0.0, 0.0), std::runtime_error);
}

TEST(KalmanFilterTest, MultipleUpdatesDoNotDiverge) {
    KalmanFilter kf;
    kf.init(1000.0, 500.0, -100.0, 20.0);

    for (int i = 0; i < 100; ++i) {
        kf.predict(0.05);
        double t = i * 0.05;
        kf.update(1000.0 - 100.0 * t, 500.0 + 20.0 * t);
    }

    // 공분산이 양수 유한값이어야 함
    for (int i = 0; i < 4; ++i) {
        EXPECT_GT(kf.covariance()(i, i), 0.0);
        EXPECT_TRUE(std::isfinite(kf.covariance()(i, i)));
    }
}
