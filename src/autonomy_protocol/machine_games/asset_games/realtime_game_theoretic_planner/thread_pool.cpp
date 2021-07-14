#include "thread_pool.h"

void threadPool::wait_function()
{
  while (true)
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    lock.unlock();
    if (terminate_pool_ )
    { break; }

    lock.lock();
    cv_.wait_for( lock, 
        std::chrono::milliseconds(100),
        [this]{return ( !this->queue_.empty() || this->terminate_pool_ );} );

      if (!queue_.empty())
      {
        std::function<void()> job = queue_.front();
        queue_.pop();
        lock.unlock();
        job();
        cv_.notify_one();
      }
  }
}

void threadPool::add_job(std::function<void()> job)
{
  std::unique_lock<std::mutex> lock(queue_mutex_);

  queue_.push(job);
  cv_.notify_one();
}

void threadPool::process_jobs()
{
  std::unique_lock<std::mutex> lock(queue_mutex_);
  lock.unlock();
  while (true)
  {
    // get lock to check queue size
    if ( lock.try_lock() )
    {
      if ( queue_.empty() )
      {
        lock.unlock();
        break;
      }
      lock.unlock();
      cv_.notify_all();
    }
  }

}

void threadPool::shutdown()
{
  process_jobs();
  shutdown_unclean();
}

void threadPool::shutdown_unclean()
{
  terminate_pool_ = true;
  cv_.notify_all();
  for  (std::thread& th : pool_)
  {
    if (th.joinable())
      th.join();
  }
  pool_.clear();

}
