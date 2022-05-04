#include <cstdlib>
#include <string>
#include <vector>
#include <memory>
#include <csignal>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sstream>

#include "yaml-cpp/yaml.h"
#include "map3d.h"

#include "warden.h"
#include "trajectory.h"
#include "trajectory_client.h"

#include "quad_state.h"
#include "quad_state_subscriber_node.h"

#include "balloon_status_subscriber_node.h"
#include "balloon_status_publisher_node.h"
#include "balloon_position_subscriber_node.h"
#include "balloon_position_publisher_node.h"
#include "balloon_status.h"

#include "goal_status_publisher_node.h"
#include "goal_status_subscriber_node.h"
#include "goal_status.h"

#include "ta-autonomy-protocol/student_autonomy_protocol.h"
#include "game_snapshot.h"
#include "map3d.h"
#include "presubmission_trajectory_vetter.h"

using namespace game_engine;

namespace {
  // Signal variable and handler
  volatile std::sig_atomic_t kill_program;
  void SigIntHandler(int sig) {
    kill_program = 1;
  }
}

int main(int argc, char** argv) {
  // Configure sigint handler
  std::signal(SIGINT, SigIntHandler);

  // Start ROS
  ros::init(argc, argv, "student_autonomy_protocol", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("/game_engine/");

  // Read ROS data
  std::string map_file_path;
  if(false == nh.getParam("map_file_path", map_file_path)) {
    std::cerr << "Required parameter not found on server: map_file_path" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  YAML::Node node;
  try {
    node = YAML::LoadFile(map_file_path);
  } catch (...) {
    std::cerr << "Map file not found.  Check map_file_path in params.yaml" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  const Map3D map3d = node["map"].as<Map3D>();

  std::map<std::string, std::string> team_assignments;
  if(false == nh.getParam("team_assignments", team_assignments)) {
    std::cerr << "Required parameter not found on server: team_assignments" << std::endl;
    std::exit(1);
  }

  std::map<std::string, std::string> quad_state_topics;
  if(false == nh.getParam("quad_state_topics", quad_state_topics)) {
    std::cerr << "Required parameter not found on server: quad_state_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::map<std::string, std::string> proposed_trajectory_topics;
  if(false == nh.getParam("proposed_trajectory_topics", proposed_trajectory_topics)) {
    std::cerr << "Required parameter not found on server: proposed_trajectory_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::vector<double> goal_position_vector;
  if(false == nh.getParam("goal_position", goal_position_vector)) {
    std::cerr << "Required parameter not found on server: goal_position" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  const Eigen::Vector3d goal_position(
      goal_position_vector[0],
      goal_position_vector[1],
      goal_position_vector[2]);

  int wind_intensity_int = 0;
  if(false == nh.getParam("wind_intensity", wind_intensity_int)) {
    std::cerr << "Required parameter not found on server: wind_intensity" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  const WindIntensity wind_intensity =
    static_cast<WindIntensity>(wind_intensity_int);

  int quad_safety_limits = 0;
  if(false == nh.getParam("quad_safety_limits", quad_safety_limits)) {
    std::cerr << "Required parameter not found on server: quad_safety_limits" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  bool joy_mode;
  if(false == nh.getParam("joy_mode", joy_mode)) {
    std::cerr << "Required parameter not found on server: joy_mode" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  bool camera_mode;
  if(false == nh.getParam("camera_mode", camera_mode)) {
    std::cerr << "Required parameter not found on server: camera_mode" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  // Team Assignments
  std::vector<std::string> red_quad_names;
  std::vector<std::string> blue_quad_names;
  for(const auto& kv: team_assignments) {
    if(kv.second == "red") {
      red_quad_names.push_back(kv.first);
    }
    if(kv.second == "blue") {
      blue_quad_names.push_back(kv.first);
    }
  }

  // Parse the initial quad positions
  std::map<std::string, std::string> initial_quad_positions_string;
  if(false == nh.getParam("initial_quad_positions", initial_quad_positions_string)) {
    std::cerr << "Required parameter not found on server: initial_quad_positions" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  std::map<
    std::string,
    Eigen::Vector3d,
    std::less<std::string>,
    Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d>>> initial_quad_positions;
  for(const auto& kv: initial_quad_positions_string) {
    const std::string& quad_name = kv.first;
    const std::string& quad_position_string = kv.second;
    std::stringstream ss(quad_position_string);
    double x,y,z;
    ss >> x >> y >> z;
    initial_quad_positions[quad_name] = Eigen::Vector3d(x,y,z);
  }

  // Initialize the QuadStateWarden. The QuadStateWarden enables safe, multi-threaded
  // access to quadcopter state data. Internal components that require access to
  // state data should request access through QuadStateWarden.
  auto quad_state_warden  = std::make_shared<QuadStateWarden>();
  for(const auto& kv: quad_state_topics) {
    const std::string& quad_name = kv.first;
    quad_state_warden->Register(quad_name);

    Eigen::Vector3d initial_quad_pos;;
    const QuadState initial_quad_state(
        (Eigen::Matrix<double, 13, 1>() <<
          initial_quad_pos(0), initial_quad_pos(1), initial_quad_pos(2),
          0,0,0,
          1,0,0,0,
          0,0,0).finished());
    
    quad_state_warden->Write(quad_name, initial_quad_state);
  }

  // Pipe ROS data into the QuadStateWarden
  std::vector<std::shared_ptr<QuadStateSubscriberNode>> quad_state_subscribers;
  for(const auto& kv: quad_state_topics) {
    const std::string& quad_name = kv.first;
    const std::string& topic = kv.second;
    quad_state_subscribers.push_back(
        std::make_shared<QuadStateSubscriberNode>(
            topic,
            quad_name,
            quad_state_warden));
  }

  // Initialize the GameSnapshot
  auto game_snapshot = std::make_shared<GameSnapshot>(
      blue_quad_names,
      red_quad_names,
      quad_state_warden,
      GameSnapshot::Options());

  auto prevetter = std::make_shared<PreSubmissionTrajectoryVetter>(quad_safety_limits, quad_state_warden);

    // Initialize the Trajectory Client
  std::unordered_map<std::string, std::shared_ptr<TrajectoryClientNode>> proposed_trajectory_clients;
  for(const auto& kv: proposed_trajectory_topics) {
    const std::string& quad_name = kv.first;
    const std::string& topic = kv.second;
    proposed_trajectory_clients[quad_name] =
      std::make_shared<TrajectoryClientNode>(topic);
  }

  // Initialize the TrajectoryWarden
  auto trajectory_warden_client = std::make_shared<TrajectoryWardenClient>();
  for(const auto& kv: proposed_trajectory_topics) {
    const std::string& quad_name = kv.first;
    trajectory_warden_client->Register(quad_name);
  }

  // Balloon Status
  std::map<std::string, std::string> balloon_status_topics;
  if(false == nh.getParam("balloon_status_topics", balloon_status_topics)) {
    std::cerr << "Required parameter not found on server: balloon_status_topics" << std::endl;
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
    if(camera_mode == true) {
      balloon_position_topic += "/estimated";
    } else {
      balloon_position_topic += "/true";
    }
  }

  // Goal Status
  std::map<std::string, std::string> goal_status_topics;
  if(false == nh.getParam("goal_status_topics", goal_status_topics)) {
    std::cerr << "Required parameter not found on server: goal_status_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  auto red_balloon_status = std::make_shared<BalloonStatus>();
  auto blue_balloon_status = std::make_shared<BalloonStatus>();

  auto red_balloon_status_subscriber_node
    = std::make_shared<BalloonStatusSubscriberNode>(balloon_status_topics["red"], red_balloon_status);
  auto blue_balloon_status_subscriber_node
    = std::make_shared<BalloonStatusSubscriberNode>(balloon_status_topics["blue"], blue_balloon_status);

  // start balloon status clock
  auto red_balloon_status_publisher_node
    = std::make_shared<BalloonStatusPublisherNode>(balloon_status_topics["red"]);
  auto blue_balloon_status_publisher_node
    = std::make_shared<BalloonStatusPublisherNode>(balloon_status_topics["blue"]);

  BalloonStatus setStartStatusRed = *(red_balloon_status_subscriber_node->balloon_status_);
  setStartStatusRed.set_start = true;

  BalloonStatus setStartStatusBlue = *(blue_balloon_status_subscriber_node->balloon_status_);
  setStartStatusBlue.set_start = true;

  // balloon position
  auto red_balloon_position = std::make_shared<Eigen::Vector3d>();
  auto blue_balloon_position = std::make_shared<Eigen::Vector3d>();

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

  // goal status
  auto goal_status = std::make_shared<GoalStatus>();
  auto goal_status_subscriber_node
    = std::make_shared<GoalStatusSubscriberNode>(goal_status_topics["home"], goal_status);
  auto goal_status_publisher_node
    = std::make_shared<GoalStatusPublisherNode>(goal_status_topics["home"]);
  GoalStatus setStartStatusGoal = *(goal_status_subscriber_node->goal_status_);
  setStartStatusGoal.set_start = true;

  // wait for .5 sec
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  red_balloon_status_publisher_node->Publish(setStartStatusRed);
  blue_balloon_status_publisher_node->Publish(setStartStatusBlue);
//  red_balloon_position_publisher_node->Publish(setStartPositionRed);
//  blue_balloon_position_publisher_node->Publish(setStartPositionBlue);
  goal_status_publisher_node->Publish(setStartStatusGoal);

  // The AutonomyProtocol
  std::shared_ptr<AutonomyProtocol> student_autonomy_protocol
    = std::make_shared<StudentAutonomyProtocol>(
      blue_quad_names,
      red_quad_names,
      game_snapshot,
      trajectory_warden_client,
      prevetter,
      map3d,
      red_balloon_status,
      red_balloon_position,
      blue_balloon_status,
      blue_balloon_position,
      goal_position,
      wind_intensity);

  // Start the autonomy protocol
  std::thread ap_thread_student(
      [&]() {
        student_autonomy_protocol->Run(proposed_trajectory_clients, joy_mode, camera_mode);
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

        student_autonomy_protocol->Stop();
        quad_state_warden->Stop();
        trajectory_warden_client->Stop();
      });

  // Spin for ros subscribers
  ros::spin();

  // Wait for program termination via ctl-c
  kill_thread.join();
  ap_thread_student.join();

  return EXIT_SUCCESS;
}
