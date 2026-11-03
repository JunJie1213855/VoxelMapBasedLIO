#pragma once
#include <atomic>
#include <condition_variable>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

namespace voxelmap_display
{
  // A bounded visualization queue. The producer never waits for serialization or DDS.
  template <typename Task>
  class LatestTaskWorker
  {
  public:
    explicit LatestTaskWorker(std::function<void(Task)> process,
                              std::function<void()> idle_process = {},
                              std::chrono::milliseconds idle_period = std::chrono::milliseconds(200))
        : process_(std::move(process)), idle_process_(std::move(idle_process)),
          idle_period_(idle_period), thread_([this]
                                             { run(); }) {}
    ~LatestTaskWorker() { stop(); }
    LatestTaskWorker(const LatestTaskWorker &) = delete;
    LatestTaskWorker &operator=(const LatestTaskWorker &) = delete;

    bool submit(Task task)
    {
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      if (!lock.owns_lock() || stopping_)
        return false;
      pending_.reset(new Task(std::move(task)));
      ready_.notify_one();
      return true;
    }
    bool idle() const
    {
      std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
      return lock.owns_lock() && !busy_ && !pending_ && !stopping_;
    }
    void stop()
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        stopping_ = true;
        pending_.reset();
      }
      ready_.notify_all();
      if (thread_.joinable())
        thread_.join();
    }
    size_t errors() const { return errors_.load(); }

  private:
    void run()
    {
      for (;;)
      {
        std::unique_ptr<Task> task;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          if (idle_process_)
            ready_.wait_for(lock, idle_period_, [this]
                            { return stopping_ || pending_; });
          else
            ready_.wait(lock, [this]
                        { return stopping_ || pending_; });
          if (stopping_)
            return;
          task = std::move(pending_);
          busy_ = true;
        }
        try
        {
          if (task)
            process_(std::move(*task));
          else if (idle_process_)
            idle_process_();
        }
        catch (...)
        {
          ++errors_;
        }
        {
          std::lock_guard<std::mutex> lock(mutex_);
          busy_ = false;
        }
      }
    }
    std::function<void(Task)> process_;
    std::function<void()> idle_process_;
    std::chrono::milliseconds idle_period_;
    mutable std::mutex mutex_;
    std::condition_variable ready_;
    std::unique_ptr<Task> pending_;
    bool busy_ = false;
    bool stopping_ = false;
    std::atomic<size_t> errors_{0};
    std::thread thread_;
  };
} // namespace voxelmap_display
