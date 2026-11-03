#include "latest_task_worker.hpp"
#include <gtest/gtest.h>
#include <future>
#include <stdexcept>
#include <vector>

namespace {
bool enqueue(pv_lio::LatestTaskWorker<int> &worker, int value) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  do {
    if (worker.submit(value)) return true;
    std::this_thread::yield();
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}
}

TEST(VisualizationWorker, BlockedPublisherDoesNotBlockProducerAndOnlyLatestPendingTaskIsKept) {
  std::promise<void> entered, release, finished;
  auto release_future = release.get_future().share();
  auto entered_future = entered.get_future();
  auto finished_future = finished.get_future();
  std::vector<int> processed;
  pv_lio::LatestTaskWorker<int> worker([&](int value) {
    if (value == 1) {
      entered.set_value();
      release_future.wait();  // emulate a slow RViz/DDS writer
    }
    processed.push_back(value);
    if (value == 3) finished.set_value();
  });
  ASSERT_TRUE(enqueue(worker, 1));
  EXPECT_EQ(entered_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_FALSE(worker.idle());
  auto producer = std::async(std::launch::async, [&] { return worker.submit(2) && worker.submit(3); });
  EXPECT_EQ(producer.wait_for(std::chrono::milliseconds(200)), std::future_status::ready);
  release.set_value();
  EXPECT_TRUE(producer.get());
  EXPECT_EQ(finished_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  worker.stop();
  EXPECT_EQ(processed, (std::vector<int>{1, 3}));
}

TEST(VisualizationWorker, AFailedVisualizationTaskDoesNotStopLaterTasks) {
  std::promise<void> started, finished;
  auto started_future = started.get_future();
  auto finished_future = finished.get_future();
  pv_lio::LatestTaskWorker<int> worker([&](int value) {
    if (value == 1) { started.set_value(); throw std::runtime_error("visualization failed"); }
    finished.set_value();
  });
  ASSERT_TRUE(enqueue(worker, 1));
  ASSERT_EQ(started_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  ASSERT_TRUE(enqueue(worker, 2));
  EXPECT_EQ(finished_future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  worker.stop();
  EXPECT_EQ(worker.errors(), 1u);
}

TEST(VisualizationWorker, CachedDataCanBeRepeatedWithoutNewSensorInput) {
  std::promise<void> repeated;
  auto future = repeated.get_future();
  std::atomic<bool> signaled{false};
  pv_lio::LatestTaskWorker<int> worker([](int) {}, [&] {
    if (!signaled.exchange(true)) repeated.set_value();
  }, std::chrono::milliseconds(10));
  EXPECT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  worker.stop();
}
