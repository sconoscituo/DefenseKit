#pragma once
#include "src/signal_processing/kalman_filter.hpp"
#include <vector>
#include <optional>

namespace defensekit {
namespace radar {

struct RadarMeasurement {
    double time;     // 측정 시각 [s]
    double range;    // 거리 [m]
    double azimuth;  // 방위각 [rad]
    double elevation;// 고각 [rad] (2D 모드 사용 시 0)
};

struct TrackState {
    int    id;
    double x, y;        // 추정 위치 [m]
    double vx, vy;      // 추정 속도 [m/s]
    double confidence;  // 추적 신뢰도 [0~1]
    int    hit_count;   // 연속 탐지 횟수
    int    miss_count;  // 연속 미탐지 횟수
    bool   confirmed;   // 확정 표적 여부 (hit >= 3)
};

// 다중 표적 추적기 (MHT: Multiple Hypothesis Tracking 단순화 버전)
class TargetTracker {
public:
    explicit TargetTracker(
        double gate_distance = 500.0,  // 게이팅 거리 [m]
        int    confirm_hits  = 3,       // 확정 필요 연속 탐지 횟수
        int    delete_misses = 5        // 삭제 연속 미탐지 횟수
    );

    // 새 측정값 처리 (프레임 단위)
    void process_measurements(
        const std::vector<RadarMeasurement>& measurements,
        double current_time
    );

    // 현재 활성 트랙 목록
    const std::vector<TrackState>& get_tracks() const { return tracks_; }

    // 확정 표적만 반환
    std::vector<TrackState> get_confirmed_tracks() const;

    // 트랙 수
    int track_count() const { return static_cast<int>(tracks_.size()); }

    // 전체 초기화
    void reset();

private:
    struct Track {
        int id;
        signal::KalmanFilter kf;
        TrackState state;
        double last_update_time;
    };

    std::vector<Track> active_tracks_;
    std::vector<TrackState> tracks_;  // 외부 노출용 캐시

    double gate_distance_;
    int    confirm_hits_;
    int    delete_misses_;
    int    next_id_{1};
    double last_time_{0.0};

    // 측정값을 직교 좌표로 변환
    static std::pair<double, double> polar_to_cartesian(
        double range, double azimuth);

    // 게이팅: 기존 트랙과 측정값 연관
    int find_nearest_track(double mx, double my) const;

    void update_track_cache();
};

} // namespace radar
} // namespace defensekit
