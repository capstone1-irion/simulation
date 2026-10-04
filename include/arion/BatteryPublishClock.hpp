#pragma once

#include <cstdint>

namespace arion {

// Pure simulation-clock schedule: no wall-clock catch-up or background work.
class BatteryPublishClock {
public:
    static constexpr int64_t kPeriodNs = 200000000;  // 5 Hz of simulation time

    // sim-time ns 기준 200ms마다 발행 허용 여부를 반환하고 시각을 저장한다. pause는 유지, 되감김은 즉시 재시작한다.
    constexpr bool due(int64_t now_ns, bool paused)
    {
        if (paused || now_ns < 0) { return false; }
        const bool rewound = initialized_ && now_ns < previous_ns_;
        previous_ns_ = now_ns;
        if (initialized_ && !rewound && now_ns - published_ns_ < kPeriodNs) { return false; }
        initialized_ = true;
        published_ns_ = now_ns;
        return true;
    }

private:
    bool initialized_{false};
    int64_t previous_ns_{0};
    int64_t published_ns_{0};
};

}  // namespace arion
