// 역할: Gazebo StringMsg.data를 변환 없이 한 줄씩 stdout에 전달하는 receive-only 어댑터.
// 입력: --topic 절대 경로, 실행별 GZ_PARTITION. 출력: data + '\n'; 진단은 stderr만 사용.
// 호출자: Gazebo보다 먼저 시작하는 Python battery_link/실행 감독자. 고장·비행 제어는 없다.

#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <gz/transport/TopicUtils.hh>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <exception>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <poll.h>
#include <string>
#include <unistd.h>
#include <utility>

namespace
{
std::atomic<bool> stopping{false};
static_assert(std::atomic<bool>::is_always_lock_free,
              "signal stop flag must be lock-free");

// signal handler에서는 할당·잠금·출력을 하지 않는다.
void Stop(int)
{
  stopping.store(true, std::memory_order_relaxed);
}

struct Inbox
{
  std::mutex mutex;
  std::condition_variable changed;
  std::deque<std::string> lines;
  std::atomic<bool> failed{false};
  bool stopped{false};
};

// stdout가 막혀도 signal로 종료할 수 있도록 작은 poll 구간으로 원문을 쓴다.
// write는 userspace 출력 버퍼를 거치지 않아 줄마다 별도 flush가 필요 없다.
bool WriteLine(const std::string &line, const std::shared_ptr<Inbox> &inbox)
{
  std::size_t offset = 0;
  while (offset < line.size() && !stopping && !inbox->failed.load())
  {
    const auto count = ::write(STDOUT_FILENO, line.data() + offset, line.size() - offset);
    if (count > 0)
    {
      offset += static_cast<std::size_t>(count);
      continue;
    }
    if (count < 0 && errno == EINTR)
      continue;
    if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    {
      pollfd output{STDOUT_FILENO, POLLOUT, 0};
      const auto result = ::poll(&output, 1, 100);
      if (result < 0 && errno == EINTR)
        continue;
      if (result >= 0 && !(output.revents & (POLLERR | POLLHUP | POLLNVAL)))
        continue;
    }
    std::cerr << "gz_feed: stdout write failed\n";
    inbox->failed.store(true);
    return false;
  }
  return offset == line.size();
}

// 종료 시 stdout의 원래 descriptor 설정을 되돌린다.
struct OutputFlags
{
  int original{-1};
  ~OutputFlags()
  {
    if (original >= 0)
      ::fcntl(STDOUT_FILENO, F_SETFL, original);
  }
};
}  // namespace

int main(int argc, char **argv)
{
  if (argc != 3 || std::string(argv[1]) != "--topic")
  {
    std::cerr << "usage: gz_feed --topic /capstone/<model>/status\n";
    return 2;
  }
  const std::string topic = argv[2];
  const char *partition = std::getenv("GZ_PARTITION");
  if (topic.empty() || topic.front() != '/' ||
      !gz::transport::TopicUtils::IsValidTopic(topic))
  {
    std::cerr << "gz_feed: --topic must be a valid absolute topic\n";
    return 2;
  }
  if (!partition || !*partition ||
      !gz::transport::TopicUtils::IsValidPartition(partition))
  {
    std::cerr << "gz_feed: a nonempty, run-isolated GZ_PARTITION is required\n";
    return 2;
  }
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  std::signal(SIGPIPE, SIG_IGN);
  OutputFlags flags;
  flags.original = ::fcntl(STDOUT_FILENO, F_GETFL);
  if (flags.original < 0 || ::fcntl(STDOUT_FILENO, F_SETFL, flags.original | O_NONBLOCK) < 0)
  {
    std::cerr << "gz_feed: cannot configure stdout\n";
    return 1;
  }

  try
  {
    const auto inbox = std::make_shared<Inbox>();
    gz::transport::Node node;
    std::function<void(const gz::msgs::StringMsg &)> callback =
        [inbox](const gz::msgs::StringMsg &message)
    {
      std::lock_guard<std::mutex> lock(inbox->mutex);
      if (inbox->stopped || inbox->failed.load())
        return;
      // 느린 소비자를 정상으로 위장하여 행을 조용히 버리지 않는다.
      if (inbox->lines.size() >= 256 || message.data().size() > 1048576)
      {
        std::cerr << "gz_feed: receive buffer limit exceeded\n";
        inbox->failed.store(true);
      }
      else
        inbox->lines.push_back(message.data() + '\n');
      inbox->changed.notify_one();
    };
    if (!node.Subscribe<gz::msgs::StringMsg>(topic, callback))
    {
      std::cerr << "gz_feed: subscribe failed for " << topic << '\n';
      return 1;
    }
    std::cerr << "gz_feed: subscribed " << topic << '\n';
    // 이 성공은 discovery 등록뿐이다. 첫 상태/침묵 제한은 Python 감독자가 책임진다.
    while (!stopping && !inbox->failed.load())
    {
      std::string line;
      {
        std::unique_lock<std::mutex> lock(inbox->mutex);
        inbox->changed.wait_for(lock, std::chrono::milliseconds(100), [&inbox]
        {
          return !inbox->lines.empty() || inbox->failed.load() || stopping;
        });
        if (stopping || inbox->failed.load())
          break;
        if (inbox->lines.empty())
          continue;
        line = std::move(inbox->lines.front());
        inbox->lines.pop_front();
      }
      if (!WriteLine(line, inbox))
        break;
    }
    {
      std::lock_guard<std::mutex> lock(inbox->mutex);
      inbox->stopped = true;
    }
    const bool unsubscribed = node.Unsubscribe(topic);
    if (!unsubscribed)
      std::cerr << "gz_feed: unsubscribe failed\n";
    return inbox->failed.load() || !unsubscribed ? 1 : 0;
  }
  catch (const std::exception &error)
  {
    std::cerr << "gz_feed: " << error.what() << '\n';
    return 1;
  }
}
