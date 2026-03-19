#include <gtest/gtest.h>
#include "src/ballistics/projectile.hpp"
#include "src/ballistics/atmosphere.hpp"
#include "src/utils/constants.hpp"
#include <cmath>

using namespace defensekit::ballistics;
using namespace defensekit::constants;

namespace {

ProjectileParams make_params(bool use_coriolis = false) {
    return {
        .mass          = 0.1,     // 100g
        .diameter      = 0.009,   // 9mm
        .drag_coeff    = 0.47,
        .use_mach_drag = true,
        .use_coriolis  = use_coriolis,
        .latitude_deg  = 37.5,
    };
}

} // namespace

TEST(AtmosphereTest, SeaLevelValues) {
    auto atm = get_atmosphere(0.0);
    EXPECT_NEAR(atm.temperature, 288.15, 0.01);
    EXPECT_NEAR(atm.pressure,    101325.0, 1.0);
    EXPECT_NEAR(atm.density,     1.225, 0.01);
}

TEST(AtmosphereTest, DensityDecreasesWithAltitude) {
    auto atm0   = get_atmosphere(0.0);
    auto atm1km = get_atmosphere(1000.0);
    auto atm5km = get_atmosphere(5000.0);
    EXPECT_GT(atm0.density, atm1km.density);
    EXPECT_GT(atm1km.density, atm5km.density);
}

TEST(ProjectileTest, TrajectoryNotEmpty) {
    ProjectileSimulator sim(make_params());
    auto traj = sim.simulate(300.0, 45.0);
    EXPECT_FALSE(traj.empty());
}

TEST(ProjectileTest, TrajectoryStartsAtOrigin) {
    ProjectileSimulator sim(make_params());
    auto traj = sim.simulate(300.0, 45.0);
    ASSERT_FALSE(traj.empty());
    EXPECT_NEAR(traj.front().pos.x, 0.0, 1e-6);
    EXPECT_NEAR(traj.front().pos.y, 0.0, 1e-6);
}

TEST(ProjectileTest, TrajectoryEndsNearGround) {
    ProjectileSimulator sim(make_params());
    auto traj = sim.simulate(300.0, 45.0);
    ASSERT_FALSE(traj.empty());
    EXPECT_NEAR(traj.back().pos.y, 0.0, 5.0); // 5m 오차 이내
}

TEST(ProjectileTest, HigherAngleReducesRange) {
    // 45도가 최대 사거리 (공기저항 있을 때는 약 38~42도)
    ProjectileSimulator sim(make_params());
    auto traj45  = sim.simulate(300.0, 45.0);
    auto traj80  = sim.simulate(300.0, 80.0);
    double range45 = traj45.back().pos.x;
    double range80 = traj80.back().pos.x;
    EXPECT_GT(range45, range80);
}

TEST(ProjectileTest, FasterMuzzleVelocityIncreasesRange) {
    ProjectileSimulator sim(make_params());
    auto traj_slow = sim.simulate(200.0, 45.0);
    auto traj_fast = sim.simulate(400.0, 45.0);
    EXPECT_GT(traj_fast.back().pos.x, traj_slow.back().pos.x);
}

TEST(ProjectileTest, SolveElevationFindsValidAngles) {
    ProjectileSimulator sim(make_params());
    double target_range = 500.0;
    double elev_low = 0.0, elev_high = 0.0;
    bool ok = sim.solve_elevation(300.0, target_range, elev_low, elev_high);
    EXPECT_TRUE(ok);
    EXPECT_GT(elev_low, 0.0);
    EXPECT_LT(elev_low, elev_high);
    EXPECT_LT(elev_high, 90.0);
}
