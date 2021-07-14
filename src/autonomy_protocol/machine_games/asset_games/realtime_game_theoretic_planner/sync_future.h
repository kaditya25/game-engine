#ifndef SYNC_FUTURE_H
#define SYNC_FUTURE_H

#include <Eigen/Core>
#include <string>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <shared_mutex>
#include <memory>
#include <atomic>

// class that protects the data member with a shared mutex and condition variable flagging
class syncFuture 
{
  public:
    std::shared_timed_mutex m_;
    std::condition_variable_any cv_;
    std::atomic<bool> failsafe_;

  public:
    syncFuture() {failsafe_=false;}

    std::vector<Eigen::VectorXd> data_;

    void reserve(const int num_vecs, const int eig_vec_size);

    void write_data(std::vector<Eigen::VectorXd>& data);

    std::shared_lock<std::shared_timed_mutex> wait_read();

    void release_write_lock(std::unique_lock<std::shared_timed_mutex>& lk);

    std::unique_lock<std::shared_timed_mutex> get_write_lock();

    std::shared_lock<std::shared_timed_mutex> get_shared_lock();

    void wait(std::shared_lock<std::shared_timed_mutex>& lk);

    std::string print();

    void write_complete();
    void reset_failsafe();
    void notify_one();
    void notify_all();
};
#endif
