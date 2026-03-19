#pragma once

namespace defensekit {
namespace constants {

// 물리 상수
constexpr double GRAVITY          = 9.80665;       // 표준 중력 가속도 [m/s^2]
constexpr double AIR_DENSITY_SEA  = 1.225;         // 해수면 공기밀도 [kg/m^3]
constexpr double EARTH_RADIUS     = 6371000.0;     // 지구 반지름 [m]
constexpr double EARTH_OMEGA      = 7.2921150e-5;  // 지구 자전 각속도 [rad/s]
constexpr double SPEED_OF_SOUND   = 343.0;         // 음속 (15°C, 해수면) [m/s]
constexpr double PI               = 3.14159265358979323846;
constexpr double DEG_TO_RAD       = PI / 180.0;
constexpr double RAD_TO_DEG       = 180.0 / PI;

// 대기 모델 상수 (국제 표준 대기 ISA)
constexpr double SEA_LEVEL_TEMP   = 288.15;        // 해수면 온도 [K]
constexpr double TEMP_LAPSE_RATE  = 0.0065;        // 기온 감률 [K/m]
constexpr double SEA_LEVEL_PRESS  = 101325.0;      // 해수면 기압 [Pa]
constexpr double GAS_CONSTANT_AIR = 287.058;       // 공기 기체상수 [J/(kg·K)]

} // namespace constants
} // namespace defensekit
