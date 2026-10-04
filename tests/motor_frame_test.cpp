#include <array>
#include <cmath>
#include <iostream>
#include <limits>

#if __has_include("arion/MotorFrame.hpp")
#include "arion/MotorFrame.hpp"
int main(int argc, char **)
{
    const std::array<float, 12> payload{{1101, 2202, 3303, 4404,
                                      15.1f, 15.2f, 15.3f, 15.4f,
                                      1.1f, 2.2f, 3.3f, 4.4f}};
    arion::MotorFrame frame{};
    if (!arion::decode_motor_frame(payload.data(), payload.size(), frame)) {
        std::cerr << "FAIL: valid four-motor payload rejected\n"; return 1;
    }
    for (std::size_t i = 0; i < 4; ++i) {
        if (frame[i].rpm != payload[i] || frame[i].voltage != payload[i + 4] ||
            frame[i].current != payload[i + 8]) {
            std::cerr << "FAIL: wrong motor/field mapping\n"; return 1;
        }
    }
    if (argc > 1) {
        auto bad = payload;
        bad[7] = std::numeric_limits<float>::quiet_NaN();
        if (arion::decode_motor_frame(bad.data(), bad.size(), frame)) {
            std::cerr << "FAIL: accepted nonfinite motor state\n"; return 1;
        }
        std::cout << "PASS: rejects nonfinite motor state\n";
    }
    if (argc > 2) {
        auto bad = payload;
        bad[0] = std::numeric_limits<float>::max();
        if (arion::decode_motor_frame(bad.data(), bad.size(), frame)) {
            std::cerr << "FAIL: accepted RPM outside int32 range\n"; return 1;
        }
        std::cout << "PASS: rejects RPM outside int32 range\n";
    }
    std::cout << "PASS: four motor identities and RPM/V/A mapping\n";
    return 0;
}
#else
int main() { std::cerr << "FAIL: motor-frame decoder is not implemented\n"; return 1; }
#endif
