#include "src/radar/target_tracker.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace defensekit {
namespace radar {

TargetTracker::TargetTracker(double gate_distance, int confirm_hits, int delete_misses)
    : gate_distance_(gate_distance)
    , confirm_hits_(confirm_hits)
    , delete_misses_(delete_misses)
{}

std::pair<double, double> TargetTracker::polar_to_cartesian(double range, double azimuth) {
    return {range * std::cos(azimuth), range * std::sin(azimuth)};
}

int TargetTracker::find_nearest_track(double mx, double my) const {
    int    best_id   = -1;
    double best_dist = gate_distance_;

    for (const auto& track : active_tracks_) {
        double dx = track.state.x - mx;
        double dy = track.state.y - my;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist < best_dist) {
            best_dist = dist;
            best_id   = track.id;
        }
    }
    return best_id;
}

void TargetTracker::process_measurements(
    const std::vector<RadarMeasurement>& measurements,
    double current_time)
{
    double dt = (last_time_ > 0.0) ? (current_time - last_time_) : 0.1;
    last_time_ = current_time;

    // 1. 모든 기존 트랙 예측
    for (auto& track : active_tracks_) {
        if (dt > 0.0)
            track.kf.predict(dt);
        track.state.x  = track.kf.x();
        track.state.y  = track.kf.y();
        track.state.vx = track.kf.vx();
        track.state.vy = track.kf.vy();
    }

    // 2. 측정값 연관 및 업데이트
    std::vector<bool> measurement_assigned(measurements.size(), false);

    for (auto& track : active_tracks_) {
        bool updated = false;
        for (int i = 0; i < static_cast<int>(measurements.size()); ++i) {
            if (measurement_assigned[i]) continue;

            auto [mx, my] = polar_to_cartesian(
                measurements[i].range, measurements[i].azimuth);

            double dx = track.state.x - mx;
            double dy = track.state.y - my;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist < gate_distance_) {
                track.kf.update(mx, my);
                track.state.x  = track.kf.x();
                track.state.y  = track.kf.y();
                track.state.vx = track.kf.vx();
                track.state.vy = track.kf.vy();
                track.state.hit_count++;
                track.state.miss_count = 0;
                track.last_update_time = current_time;
                measurement_assigned[i] = true;
                updated = true;
                break;
            }
        }
        if (!updated) {
            track.state.miss_count++;
        }
        // 확정 여부 갱신
        track.state.confirmed = (track.state.hit_count >= confirm_hits_);
        // 신뢰도: 연속 탐지 비율
        int total = track.state.hit_count + track.state.miss_count;
        track.state.confidence = total > 0
            ? static_cast<double>(track.state.hit_count) / total
            : 0.0;
    }

    // 3. 미연관 측정값 → 신규 트랙 생성
    for (int i = 0; i < static_cast<int>(measurements.size()); ++i) {
        if (measurement_assigned[i]) continue;

        auto [mx, my] = polar_to_cartesian(
            measurements[i].range, measurements[i].azimuth);

        Track new_track;
        new_track.id = next_id_++;
        new_track.kf.init(mx, my, 0.0, 0.0);
        new_track.state = {new_track.id, mx, my, 0.0, 0.0, 0.0, 1, 0, false};
        new_track.last_update_time = current_time;
        active_tracks_.push_back(std::move(new_track));
    }

    // 4. 소멸 조건 트랙 제거
    active_tracks_.erase(
        std::remove_if(active_tracks_.begin(), active_tracks_.end(),
            [this](const Track& t) {
                return t.state.miss_count >= delete_misses_;
            }),
        active_tracks_.end()
    );

    update_track_cache();
}

std::vector<TrackState> TargetTracker::get_confirmed_tracks() const {
    std::vector<TrackState> confirmed;
    for (const auto& t : tracks_)
        if (t.confirmed) confirmed.push_back(t);
    return confirmed;
}

void TargetTracker::reset() {
    active_tracks_.clear();
    tracks_.clear();
    next_id_ = 1;
    last_time_ = 0.0;
}

void TargetTracker::update_track_cache() {
    tracks_.clear();
    tracks_.reserve(active_tracks_.size());
    for (const auto& t : active_tracks_)
        tracks_.push_back(t.state);
}

} // namespace radar
} // namespace defensekit
