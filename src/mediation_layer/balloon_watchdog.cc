#include <chrono>
#include <thread>

#include <ros/ros.h>
#include "balloon_watchdog.h"
#include "balloon_status.h"

namespace game_engine {
  void BalloonWatchdog::Run(
      std::shared_ptr<BalloonStatusPublisherNode> balloon_status_publisher,
      std::shared_ptr<BalloonStatusSubscriberNode> balloon_status_subscriber,
      std::shared_ptr<QuadStateWarden> quad_state_warden,
      const std::vector<std::string>& quad_names,
      Eigen::Vector3d& balloon_position,
      Eigen::Vector3d& new_balloon_position,
      double max_move_time,
      std::mt19937& gen,
      const std::string& topic) {

    // Data to be populated when balloon is popped
    bool balloon_popped = false;
    std::chrono::system_clock::time_point start_time = std::chrono::system_clock::now();
      
    std::uniform_int_distribution<int> distribution(0.0, max_move_time);
    double move_time = distribution(gen);

    double balloon_pop_time = -1.0; // initial (invalid) time before popping
    std::string quad_popper = "null";

    bool set_start;
    bool started = false; // set to true after SAP starts

    Eigen::Vector3d position = balloon_position;


    std::cout << "Setting up subscriber" << std::endl;
    ros::NodeHandle node_handle_ = ros::NodeHandle("/game_engine/");
    ros::Subscriber subscriber_ = node_handle_.subscribe(
        topic, 
        1, 
        &BalloonWatchdog::ManualCallback, 
        this);

    // Main loop
    while(true == this->ok_) {
      // read start time from existing balloon_status
      set_start = balloon_status_subscriber->balloon_status_->set_start;
      if (set_start && !started){
        // resets clock after SAP starts
        start_time = std::chrono::system_clock::now();
        started = true;
      }

      if (started) {
        auto now = std::chrono::system_clock::now();
        std::chrono::duration<double> difference = now - start_time;
        double elapsed_sec = difference.count();

        // move balloon if enough time has passed
        if (elapsed_sec >= move_time) {
          position = new_balloon_position;
        }
      
        for(const std::string& quad_name: quad_names) {
          QuadState quad_state;
          quad_state_warden->Read(quad_name, quad_state);
	
          const Eigen::Vector3d quad_pos = quad_state.Position();
          const double distance_to_balloon = (quad_pos - position).norm();

          if(manualPop || this->options_.pop_distance >= distance_to_balloon) {
            if(false == balloon_popped) {
              ROS_INFO_STREAM("Balloon popped @ elapsed: " << elapsed_sec);
              balloon_popped = true;
              
              balloon_pop_time = elapsed_sec;
              quad_popper = quad_name;
            }
          }
        }
      }

      // Publish
      BalloonStatus balloon_status {
        .popped = balloon_popped,
        .popper = quad_popper,
        .pop_time = balloon_pop_time,
        .position = position,
        .set_start = set_start // only set to true from SAP
      };

      balloon_status_publisher->Publish(balloon_status);

      // 50 Hz
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void BalloonWatchdog::Stop() {
    this->ok_ = false;
  }


  void BalloonWatchdog::ManualCallback(const std_msgs::Bool& msg) {
  	if(msg.data) {this->manualPop = true; 
  	ROS_WARN("Received manual pop request");
  	}
  }
}
