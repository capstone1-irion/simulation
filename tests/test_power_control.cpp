#include "arion/PowerControl.hpp"

#include <iostream>
#include <functional>
#include <memory>
#include <atomic>
#include <thread>
#include <stdexcept>
#include <string>

namespace {
// 실패 조건과 설명을 받아 좁은 단위시험을 중단한다(제품 상태 변경 없음).
void expect(bool value, const std::string &message)
{
    if (!value) { throw std::runtime_error(message); }
}
// 허용 명령 문법과 숫자/ID/추가 토큰 거부를 검사한다(시뮬레이터 실행 없음).
void strict_commands()
{
    arion::PowerControlCommand command;
    std::string error;
    for (const std::string text : {"status", " clear\t", "reset", "restore", "profile normal-1",
                                  "fault 0 0 0", "fault 3 3 1", "fault 1 2.25 0.5"}) {
        expect(arion::parse_power_command(text, command, error), "accept: " + text);
    }
    for (const std::string text : {"", "STATUS", "status extra", "clear 1", "reset now", "profile",
                                  "profile a b", "profile a\"b", "fault 4 1 1", "fault -1 1 1",
                                  "fault 0x1 1 1", "fault 1 3.1 1", "fault 1 1 1.1", "fault 1 -1 1",
                                  "fault 1 nan 1", "fault 1 inf 1", "fault 1 1e999 1", "fault 1 1x 1",
                                  "fault 1 1 1 extra", "fault 1 1"}) {
        expect(!arion::parse_power_command(text, command, error), "reject: " + text);
    }
    expect(!arion::parse_power_command(std::string("status\0junk", 11), command, error), "reject embedded NUL");
    expect(!arion::parse_power_command(std::string(257, ' '), command, error), "bound request size");
    expect(arion::power_json_error("quote\"\\\n\x01") ==
           "{\"ok\":false,\"error\":\"quote\\\"\\\\\\n\\u0001\"}", "JSON escapes controls and quotes");
}
}  // namespace

// 수신 승인과 sim-thread 적용을 구분하고 pause·reset·FIFO 상태 일치를 검사한다.
void queued_control()
{
    arion::CoupledPowerModel model;
    arion::PowerControlMailbox mailbox(model);
    expect(mailbox.handle("fault 2 2.5 0.2") == "{\"ok\":true,\"accepted_revision\":1}", "accept revision");
    expect(model.current_scales()[2] == 1.0, "callback does not mutate model");
    expect(mailbox.snapshot().applied_revision == 0, "accepted is not applied");
    mailbox.apply_pending(model, 100, true);
    expect(mailbox.snapshot().applied_revision == 0 && mailbox.snapshot().pending_commands == 1,
           "pause holds commands");
    expect(mailbox.handle("status").find("\"paused\":true") != std::string::npos, "status while paused");
    mailbox.apply_pending(model, 200, false);
    expect(mailbox.snapshot().applied_revision == 1 && model.current_scales()[2] == 2.5,
           "simulation thread applies accepted fault");
    expect(mailbox.snapshot().motor_rpm_scales[2] == 0.2, "status matches applied model");
    expect(mailbox.handle("profile missing").find("\"ok\":false") != std::string::npos, "unknown profile rejected");
    mailbox.handle("clear");
    mailbox.handle("fault 1 3 0");
    mailbox.handle("reset");
    expect(mailbox.apply_pending(model, 300, false), "reset signals schedule invalidation");
    expect(model.current_scales()[1] == 1.0 && mailbox.snapshot().applied_revision == 4,
           "FIFO reset clears earlier faults");
    expect(mailbox.snapshot().reset_count == 1 && mailbox.snapshot().last_sample_ns == -1,
           "reset increments generation and invalidates freshness");
    mailbox.handle("fault 0 2 0.5");
    mailbox.request_world_reset();
    mailbox.handle("fault 3 2 0.5");
    expect(mailbox.apply_pending(model, 0, true), "world reset applies even if paused");
    expect(mailbox.snapshot().applied_revision == 4 && mailbox.snapshot().cancelled_through_revision == 5 &&
           mailbox.snapshot().pending_commands == 1, "reset cancels pre-reset queue, never fakes application");
    expect(model.current_scales()[0] == 1.0 && model.current_scales()[3] == 1.0, "world reset clears faults");
    mailbox.apply_pending(model, 1, false);
    expect(mailbox.snapshot().applied_revision == 6 && model.current_scales()[3] == 2.0,
           "post-reset command preserved with monotonic revision");
    auto shared = std::make_shared<arion::PowerControlMailbox>(model);
    // 지연 dispatch 콜백이 owner를 참조하지 않는 수명 경계를 재현한다.
    std::function<std::string()> delayed = [shared] { return shared->handle("status"); };
    shared->close();
    shared.reset();
    expect(delayed().find("\"ok\":false") != std::string::npos, "closed shared mailbox safe after owner release");
}

// 실제 콜백/상태 읽기 스레드와 단일 적용 스레드를 경합시켜 큐/리비전의 일관성을 검사한다.
void concurrent_mailbox()
{
    arion::CoupledPowerModel model;
    arion::PowerControlMailbox mailbox(model);
    for (int i = 0; i < 128; ++i) {
        expect(mailbox.handle("clear").find("\"ok\":true") != std::string::npos, "bounded queue accepts capacity");
    }
    expect(mailbox.handle("clear").find("\"ok\":false") != std::string::npos, "queue overflow rejected");
    expect(mailbox.snapshot().accepted_revision == 128, "rejection does not advance revision");
    mailbox.apply_pending(model, 1, false);
    std::atomic<int> producers{2};
    std::atomic<bool> consistent{true};
    // 콜백 스레드는 mailbox에만 접근하고 총 40개의 명령을 승인받는다.
    const auto writer = [&mailbox, &producers, &consistent] {
        for (int i = 0; i < 40; ++i) {
            if (mailbox.handle("fault 0 2 0.5").find("\"ok\":true") == std::string::npos) { consistent = false; }
        }
        --producers;
    };
    std::thread one(writer);
    std::thread two(writer);
    // 별도 읽기 스레드는 모델 대신 동기화된 상태 복사본과 JSON만 읽는다.
    std::thread reader([&mailbox, &producers, &consistent] {
        do {
            const auto snapshot = mailbox.snapshot();
            if (snapshot.applied_revision > snapshot.accepted_revision || snapshot.pending_commands > 128 ||
                mailbox.handle("status").find("\"ok\":true") == std::string::npos) { consistent = false; }
        } while (producers != 0);
    });
    int64_t ns = 2;
    while (producers != 0) { mailbox.apply_pending(model, ns++, false); std::this_thread::yield(); }
    one.join();
    two.join();
    reader.join();
    mailbox.apply_pending(model, ns, false);
    expect(consistent && mailbox.snapshot().applied_revision == 128 + 2 * 40, "concurrent FIFO/revision consistency");
}

// 순수 제어 및 스레드 경계 시험을 실행하며 외부 시뮬레이터를 시작하지 않는다.
int main()
{
    try {
        strict_commands();
        std::cout << "PASS strict_commands\n";
        queued_control();
        std::cout << "PASS queued_control\n";
        concurrent_mailbox();
        std::cout << "PASS concurrent_mailbox\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
