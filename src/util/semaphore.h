
#pragma once

#include <mutex>
#include <condition_variable>

namespace game_engine {
  class Semaphore {
    private:
      std::mutex mtx_;
      std::condition_variable cv_;
      int count_;

    public:
      Semaphore(int count = 0) : count_{count} {}

      void Notify();

      void Wait();
  };
}
