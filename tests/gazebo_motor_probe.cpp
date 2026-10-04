#include <gz/msgs/float_v.pb.h>
#include <gz/transport/Node.hh>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>

// Independent expected values: do not import the producer's constants.
int main(int argc, char **argv)
{
    if (argc != 2 && argc != 3) { std::cerr << "usage: gazebo_motor_probe TOPIC [cadence]\n"; return 2; }
    const bool check_cadence = argc == 3 && std::string(argv[2]) == "cadence";
    struct CallbackState {
        const std::array<float, 12> expected{{1101, 2202, 3303, 4404,
                                            15.1f, 15.2f, 15.3f, 15.4f,
                                            1.1f, 2.2f, 3.3f, 4.4f}};
        std::atomic<unsigned> valid{0}, invalid{0};
        std::mutex print_mutex;
        int64_t previous_stamp = -1;
    };
    const auto state = std::make_shared<CallbackState>();
    gz::transport::Node node;
    // Dispatch may retain this function beyond Unsubscribe and main's stack.
    std::function<void(const gz::msgs::Float_V &)> callback = [state, check_cadence](const auto &msg) {
        std::lock_guard<std::mutex> guard(state->print_mutex);
        bool schema_ok = false;
        for (const auto &entry : msg.header().data()) {
            if (entry.key() == "schema" && entry.value_size() == 1 &&
                entry.value(0) == "arion.motor_state.v1") { schema_ok = true; }
        }
        bool ok = schema_ok && msg.data_size() == static_cast<int>(state->expected.size());
        if (ok) {
            for (int i = 0; i < msg.data_size(); ++i) {
                ok = ok && std::isfinite(msg.data(i)) &&
                     std::abs(msg.data(i) - state->expected[static_cast<std::size_t>(i)]) < 0.001f;
            }
        }
        const int64_t stamp = msg.header().stamp().sec() * 1000000000 + msg.header().stamp().nsec();
        if (check_cadence && state->previous_stamp >= 0 &&
            std::abs(stamp - state->previous_stamp - 50000000) > 10000000) {
            std::cerr << "wrong_simulated_period_ns=" << stamp - state->previous_stamp << '\n';
            ok = false;
        }
        state->previous_stamp = stamp;
        if (!ok) { ++state->invalid; return; }
        if (state->valid.fetch_add(1) == 0) {
            std::cout << "FIRST_ACTUAL_GAZEBO_FRAME\n" << msg.DebugString() << std::flush;
        }
    };
    if (!node.Subscribe(argv[1], callback)) { std::cerr << "subscription failed\n"; return 2; }
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(6);
    while (std::chrono::steady_clock::now() < deadline && state->valid.load() < 5 && state->invalid.load() == 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    node.Unsubscribe(argv[1]);
    std::cout << "valid_frames=" << state->valid.load() << " invalid_frames=" << state->invalid.load() << '\n';
    if (state->valid.load() < 5 || state->invalid.load() != 0) {
        std::cerr << "FAIL: four-motor fixture was not received from the Gazebo plugin\n";
        return 1;
    }
    return 0;
}
