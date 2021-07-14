#ifndef THREAD_POOL_H
#define THREAD_POOL_H
#include <iostream>
#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>
#include <atomic>
#include <memory>
#include <chrono>

class threadPool
{
  std::mutex queue_mutex_;
  std::condition_variable cv_;
  bool terminate_pool_ = false;
  std::queue<std::function<void()>> queue_;

  public:
  std::vector<std::thread> pool_;

  threadPool(int num_threads)
  {
    pool_ = std::vector<std::thread>(num_threads);
    for (int i=0; i<num_threads; i++)
    {
      pool_.emplace_back( &threadPool::wait_function,this );
    }
  }

  void wait_function();

  void add_job(std::function<void()> task);

  void shutdown();

  void shutdown_unclean();

  void process_jobs();

  ~threadPool()
  { shutdown_unclean(); }


};

#endif
