#include <chrono>
#include <thread>

#include "goal_watchdog.h"
#include "goal_status.h"

namespace game_engine {
  void GoalWatchdog::Run(
      std::shared_ptr<GoalStatusPublisherNode> goal_status_publisher,
      std::shared_ptr<GoalStatusSubscriberNode> goal_status_subscriber,
      std::shared_ptr<QuadStateWarden> quad_state_warden,
      const std::vector<std::string>& quad_names,
      Eigen::Vector3d& goal_position) {

    // Data to be populated when goal is reached
    bool active = false;
    bool goal_reached = false;
    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();

    double goal_reach_time = -1.0; // initial (invalid) time before popping
    std::string quad_scorer = "null";

    bool set_start;
    bool started = false; // set to true after SAP starts

    // Main loop
    while(true == this->ok_) {
      // read start time from existing goal status
      set_start = goal_status_subscriber->goal_status_->set_start;
      if (set_start && !started){
        // resets clock after SAP starts
        start_time = std::chrono::system_clock::now();
        started = true;
      }

      if (started) {
        auto now = std::chrono::system_clock::now();
        std::chrono::duration<double> difference = now - start_time;
        double elapsed_sec = difference.count();
      
        for(const std::string& quad_name: quad_names) {
          QuadState quad_state;
          quad_state_warden->Read(quad_name, quad_state);

          const Eigen::Vector3d quad_pos = quad_state.Position();
          const double quad_speed = (quad_state.Velocity()).norm();
          const double distance_to_goal = (quad_pos - goal_position).norm();

          if(this->options_.reach_distance >= distance_to_goal && quad_speed <= this->options_.reach_speed) {
            if(false == goal_reached && elapsed_sec >= this->options_.time_fuze) {
              goal_reached = true;
              active = true;
              
              goal_reach_time = elapsed_sec;
              quad_scorer = quad_name;
            } else {
              active = true;
            }
          } else {
            active = false;
          }
        }
      }

      // Publish
      GoalStatus goal_status {
        .active = active,
        .reached = goal_reached,
        .scorer = quad_scorer,
        .reach_time = goal_reach_time,
        .position = goal_position,
        .set_start = set_start // only set to true from SAP
      };

      goal_status_publisher->Publish(goal_status);

      // 50 Hz
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void GoalWatchdog::Stop() {
    this->ok_ = false;
  }

}
