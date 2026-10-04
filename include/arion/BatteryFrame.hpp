#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace arion {

inline constexpr char kBatterySchema[] = "arion.battery_state.v1";
// ID, cell 1..4 volts, discharge amps, remaining fraction, degC, consumed mAh.
// Deliberate transport fixture, not a measured or calibrated battery pack.
inline constexpr std::array<float, 9> kFixedBatteryValues{
    0.0f, 4.01f, 4.02f, 4.03f, 4.04f, 7.25f, 0.73f, 28.5f, 123.0f};

struct BatteryWireValues {
    uint8_t id{0};
    std::array<uint16_t, 10> voltages{
        65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535};
    int16_t current_ca{-1};
    int8_t remaining_pct{-1};
    int16_t temperature_cdeg{32767};
    int32_t consumed_mah{-1};
};

// Range checks also reject NaN/Inf before a floating-to-integer conversion.
// Quantize to nearest wire unit, with half values rounded away from zero.
// 물리량을 지정 배율의 wire 정수로 반올림한다. 비유한/범위 밖 입력이면 out을 보존하고 false를 반환한다.
constexpr bool battery_wire_integer(float value, double scale,
                                    int32_t minimum, int32_t maximum, int32_t &out)
{
    const double scaled = static_cast<double>(value) * scale;
    if (!(scaled >= minimum && scaled <= maximum)) { return false; }
    out = static_cast<int32_t>(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
    return true;
}

// This v1 contract is one battery (ID 0), four known cells, discharge only.
// Unknown/nonfinite active fields are rejected, not silently replaced with zero.
// Keep the caller's previous output unchanged if any field is invalid.
// ID0·4셀V·팩A·SOC비율·섭씨·소비mAh의 9필드를 MAVLink 단위로 변환한다. 전체 검증 후에만 output을 갱신한다.
constexpr bool decode_battery_frame(const float *data, std::size_t count,
                                    BatteryWireValues &output)
{
    if (!data || count != kFixedBatteryValues.size() || data[0] != 0.0f) { return false; }
    BatteryWireValues candidate{};
    int32_t integer = 0;
    for (std::size_t cell = 0; cell < 4; ++cell) {
        if (!battery_wire_integer(data[cell + 1], 1000.0, 1, 65534, integer)) { return false; }
        candidate.voltages[cell] = static_cast<uint16_t>(integer);
    }
    if (!battery_wire_integer(data[5], 100.0, 0, 32767, integer)) { return false; }
    candidate.current_ca = static_cast<int16_t>(integer);
    if (!battery_wire_integer(data[6], 100.0, 0, 100, integer)) { return false; }
    candidate.remaining_pct = static_cast<int8_t>(integer);
    if (!battery_wire_integer(data[7], 100.0, -32768, 32766, integer)) { return false; }
    candidate.temperature_cdeg = static_cast<int16_t>(integer);
    if (!battery_wire_integer(data[8], 1.0, 0, std::numeric_limits<int32_t>::max(), integer)) {
        return false;
    }
    candidate.consumed_mah = integer;
    output = candidate;
    return true;
}

}  // namespace arion
