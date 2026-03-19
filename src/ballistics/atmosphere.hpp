#pragma once
#include "src/utils/constants.hpp"
#include <cmath>

namespace defensekit {
namespace ballistics {

// 국제 표준 대기 모델 (ISA) - 대류권 (0~11km)
struct AtmosphereState {
    double temperature;   // [K]
    double pressure;      // [Pa]
    double density;       // [kg/m^3]
    double speed_of_sound; // [m/s]
};

inline AtmosphereState get_atmosphere(double altitude_m) {
    using namespace constants;

    // 11km 이상은 성층권 근사
    const double alt = std::max(0.0, std::min(altitude_m, 86000.0));

    AtmosphereState state{};

    if (alt <= 11000.0) {
        // 대류권
        state.temperature = SEA_LEVEL_TEMP - TEMP_LAPSE_RATE * alt;
        state.pressure    = SEA_LEVEL_PRESS *
            std::pow(state.temperature / SEA_LEVEL_TEMP,
                     GRAVITY / (GAS_CONSTANT_AIR * TEMP_LAPSE_RATE));
    } else {
        // 성층권 하부 (등온)
        constexpr double T11 = SEA_LEVEL_TEMP - TEMP_LAPSE_RATE * 11000.0;
        constexpr double P11 = SEA_LEVEL_PRESS *
            std::pow(T11 / SEA_LEVEL_TEMP,
                     GRAVITY / (GAS_CONSTANT_AIR * TEMP_LAPSE_RATE));
        state.temperature = T11;
        state.pressure    = P11 * std::exp(-GRAVITY * (alt - 11000.0) /
                                           (GAS_CONSTANT_AIR * T11));
    }

    state.density       = state.pressure / (GAS_CONSTANT_AIR * state.temperature);
    state.speed_of_sound = std::sqrt(1.4 * GAS_CONSTANT_AIR * state.temperature);
    return state;
}

// 마하수 계산
inline double mach_number(double velocity_ms, double altitude_m) {
    return velocity_ms / get_atmosphere(altitude_m).speed_of_sound;
}

// 항력 계수 (마하수에 따른 근사 모델)
inline double drag_coefficient(double mach) {
    if (mach < 0.8)        return 0.20;
    else if (mach < 1.0)   return 0.20 + 0.35 * (mach - 0.8) / 0.2;
    else if (mach < 1.2)   return 0.55 - 0.15 * (mach - 1.0) / 0.2;
    else if (mach < 2.0)   return 0.40 - 0.10 * (mach - 1.2) / 0.8;
    else                   return 0.30;
}

} // namespace ballistics
} // namespace defensekit
