#include <cstdlib>
#include <string>
#include <vector>
#include <memory>
#include <csignal>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sstream>
#include <thread>

#include "yaml-cpp/yaml.h"

#include "balloon_position_subscriber_node.h"
#include "balloon_position_publisher_node.h"
#include "balloon_status.h"

#include "goal_status_publisher_node.h"
#include "goal_status_subscriber_node.h"
#include "goal_status.h"

using namespace game_engine;

namespace {
    // Signal variable and handler
    volatile std::sig_atomic_t kill_program;
    void SigIntHandler(int sig) {
      kill_program = 1;
    }
}

class CameraHandler {
protected:
    volatile std::atomic<bool> ok_{true};

public:
    CameraHandler() {}

    virtual ~CameraHandler() {}

    // Stop this thread from running
    void Stop();

    // Main loop for this thread
    void Run(std::shared_ptr<BalloonPositionPublisherNode> blue_balloon_position_publisher,
             Eigen::Vector3d setStartPositionBlue,
             std::shared_ptr<BalloonPositionPublisherNode> red_balloon_position_publisher,
             Eigen::Vector3d setStartPositionRed);
};

inline void CameraHandler::Stop() {
  ok_ = false;
}
inline void CameraHandler::Run(std::shared_ptr<BalloonPositionPublisherNode> blue_balloon_position_publisher,
                               Eigen::Vector3d setStartPositionBlue,
                               std::shared_ptr<BalloonPositionPublisherNode> red_balloon_position_publisher,
                               Eigen::Vector3d setStartPositionRed) {
  while(ok_) {
    blue_balloon_position_publisher->Publish(setStartPositionBlue);
    red_balloon_position_publisher->Publish(setStartPositionRed);
    std::cout << "red position: " << setStartPositionRed << std::endl;
    std::cout << "blue position: " << setStartPositionBlue << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

int main(int argc, char** argv) {
  // Configure sigint handler
  std::signal(SIGINT, SigIntHandler);

  // Start ROS
  ros::init(argc, argv, "test_camera", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("/game_engine/");

  bool camera_mode;
  if(false == nh.getParam("camera_mode", camera_mode)) {
    std::cerr << "Required parameter not found on server: camera_mode" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  // Balloon Position
  std::map<std::string, std::string> balloon_position_topics;
  if(false == nh.getParam("balloon_position_topics", balloon_position_topics)) {
    std::cerr << "Required parameter not found on server: balloon_position_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  for(auto& kv: balloon_position_topics) {
    std::string &balloon_position_topic = kv.second;
    balloon_position_topic += "/estimated";
    std::cout << "topic: " << balloon_position_topic << std::endl;
  }

  // balloon position
  auto red_balloon_position = std::make_shared<Eigen::Vector3d>(-26, 19, -3);
  auto blue_balloon_position = std::make_shared<Eigen::Vector3d>(-18, 19, -5);

  auto red_balloon_position_subscriber_node
      = std::make_shared<BalloonPositionSubscriberNode>(balloon_position_topics["red"], red_balloon_position);
  auto blue_balloon_position_subscriber_node
      = std::make_shared<BalloonPositionSubscriberNode>(balloon_position_topics["blue"], blue_balloon_position);

  auto red_balloon_position_publisher_node
      = std::make_shared<BalloonPositionPublisherNode>(balloon_position_topics["red"]);
  auto blue_balloon_position_publisher_node
      = std::make_shared<BalloonPositionPublisherNode>(balloon_position_topics["blue"]);

  Eigen::Vector3d setStartPositionRed = *(red_balloon_position_subscriber_node->balloon_position_);
  Eigen::Vector3d setStartPositionBlue = *(blue_balloon_position_subscriber_node->balloon_position_);

  // wait for .5 sec
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  // The Camera Handler
  std::shared_ptr<CameraHandler> camera_handler = std::make_shared<CameraHandler>();

  // Start the autonomy protocol
  std::thread camera_thread_example(
      [&]() {
          camera_handler->Run(blue_balloon_position_publisher_node, setStartPositionBlue,
                              red_balloon_position_publisher_node, setStartPositionRed);
      });

  // Start the kill thread
  std::thread kill_thread(
      [&]() {
          while(true) {
            if(true == kill_program) {
              break;
            } else {
              std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
          }

          ros::shutdown();
          camera_handler->Stop();
      });

  // Spin for ros subscribers
  ros::spin();

  // Wait for program termination via ctl-c
  kill_thread.join();
  camera_thread_example.join();

  return EXIT_SUCCESS;
}
