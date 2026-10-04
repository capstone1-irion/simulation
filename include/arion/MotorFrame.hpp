#pragma once

#include <array>
#include <cstddef>
#include <cmath>
#include <cstdint>
#include <limits>

namespace arion {
struct MotorState { float rpm; float voltage; float current; };
using MotorFrame = std::array<MotorState, 4>;

// RPM4→V4→A4의 12필드를 네 모터 구조체로 옮긴다. 유한값·RPM 정수범위를 검사하며 실패 시 output은 보존한다.
inline bool decode_motor_frame(const float *data, std::size_t count, MotorFrame &output)
{
    if (!data || count != 12) { return false; }
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(data[i])) { return false; }
    }
    for (std::size_t i = 0; i < output.size(); ++i) {
        const double rpm = static_cast<double>(data[i]);
        if (rpm < static_cast<double>(std::numeric_limits<int32_t>::min()) ||
            rpm > static_cast<double>(std::numeric_limits<int32_t>::max())) { return false; }
    }
    for (std::size_t i = 0; i < output.size(); ++i) {
        output[i] = MotorState{data[i], data[i + 4], data[i + 8]};
    }
    return true;
}
}  // namespace arion
