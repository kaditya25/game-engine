#include "semaphore.h"

namespace game_engine {
  void Semaphore::Notify() {
  std::unique_lock<std::mutex> lck(mtx_);
  ++ count_;
  cv_.notify_one();
  }

  void Semaphore::Wait() {
    std::unique_lock<std::mutex> lck(mtx_);
    while(count_ == 0) {
      cv_.wait(lck);
    }
    --count_;
  }
}
