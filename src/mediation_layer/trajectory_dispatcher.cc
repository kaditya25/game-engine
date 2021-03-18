// Author: Tucker Haydon

#include "trajectory_dispatcher.h"

namespace game_engine {
  void TrajectoryDispatcher::Run(
      std::shared_ptr<TrajectoryWarden> warden,
      std::unordered_map<
        std::string,
        std::shared_ptr<TrajectoryClientNode>> trajectory_clients) {
    // Local thread pool
    std::vector<std::thread> thread_pool;

    // Get all registered trajectories
    const std::set<std::string> trajectory_keys = warden->Keys();

    // Assign a thread to await changes in each trajectory
    for(const std::string& key: trajectory_keys) {
      thread_pool.push_back(
          std::move(
            std::thread([&](){
              this->AwaitTrajectoryChange(key, warden, trajectory_clients[key]);
              })));
    }

    // Wait for this thread to receive a stop command
    std::thread kill_thread(
        [&, this]() {
          while(true) {
            if(false == this->ok_) {
              break;
            } else {
              std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
          }
        });

    kill_thread.join();

    // Wait for thread pool to terminate
    for(std::thread& t: thread_pool) {
      t.join();
    }
  }

  void TrajectoryDispatcher::AwaitTrajectoryChange(
      const std::string key,
      std::shared_ptr<TrajectoryWarden> warden,
      std::shared_ptr<TrajectoryClientNode> client) {
    while(this->ok_) {
      Trajectory trajectory;
      if(Success == warden->Await(key, trajectory)) {
        // Check the value of the response from the client-server
        unsigned int status = client->Call(trajectory);
        // Update the status in the warden
        warden->SetTrajectoryStatus(status);
      }
    }
  }


  void TrajectoryDispatcher::Stop() {
    this->ok_ = false;
  }
}
