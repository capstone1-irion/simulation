// Compile-only contract checks: this file never starts a simulator or socket.
#if __has_include("arion/BatteryFrame.hpp")
#include "arion/BatteryFrame.hpp"

constexpr bool fixed_battery_maps_to_mavlink_units()
{
    arion::BatteryWireValues out{};
    const auto &values = arion::kFixedBatteryValues;
    if (!arion::decode_battery_frame(values.data(), values.size(), out)) { return false; }
    return out.id == 0 && out.voltages[0] == 4010 && out.voltages[1] == 4020 &&
           out.voltages[2] == 4030 && out.voltages[3] == 4040 &&
           out.voltages[4] == 65535 && out.voltages[9] == 65535 &&
           out.current_ca == 725 && out.remaining_pct == 73 &&
           out.temperature_cdeg == 2850 && out.consumed_mah == 123;
}
static_assert(fixed_battery_maps_to_mavlink_units(),
              "Battery ID, four cell voltages, current, SOC, temperature or consumption mapping failed");

constexpr bool invalid_frames_leave_output_unchanged()
{
    arion::BatteryWireValues out{};
    if (arion::decode_battery_frame(nullptr, 9, out) ||
        arion::decode_battery_frame(arion::kFixedBatteryValues.data(), 8, out)) { return false; }
    for (std::size_t i = 0; i < arion::kFixedBatteryValues.size(); ++i) {
        auto bad = arion::kFixedBatteryValues;
        bad[i] = std::numeric_limits<float>::quiet_NaN();
        if (arion::decode_battery_frame(bad.data(), bad.size(), out)) { return false; }
        bad[i] = std::numeric_limits<float>::infinity();
        if (arion::decode_battery_frame(bad.data(), bad.size(), out)) { return false; }
    }
    const std::array<std::size_t, 7> slots{0, 1, 2, 5, 6, 7, 8};
    const std::array<float, 7> bad_values{1, 0, 70, -1, 1.01f, 400, 2147483648.0f};
    for (std::size_t i = 0; i < slots.size(); ++i) {
        auto bad = arion::kFixedBatteryValues;
        bad[slots[i]] = bad_values[i];
        if (arion::decode_battery_frame(bad.data(), bad.size(), out)) { return false; }
    }
    return out.voltages[0] == 65535 && out.current_ca == -1 &&
           out.remaining_pct == -1 && out.temperature_cdeg == 32767 && out.consumed_mah == -1;
}
static_assert(invalid_frames_leave_output_unchanged(), "Invalid input accepted or partial output escaped");

#if __has_include("arion/BatteryPublishClock.hpp")
#include "arion/BatteryPublishClock.hpp"
constexpr bool publishing_uses_simulation_time()
{
    arion::BatteryPublishClock clock;
    return !clock.due(-1, false) && !clock.due(0, true) &&
           clock.due(0, false) && !clock.due(0, false) &&
           !clock.due(199999999, false) && clock.due(200000000, false) &&
           !clock.due(400000000, true) && clock.due(400000000, false) &&
           clock.due(0, false) && !clock.due(0, false);
}
static_assert(publishing_uses_simulation_time(), "Pause, 5 Hz cadence or reset handling failed");
#else
static_assert(false, "Battery publication clock is not implemented");
#endif
#else
static_assert(false, "Battery frame contract is not implemented");
#endif
