#include <cstdlib>
#include <vector>
#include <string>
#include <map>
#include <string>
#include <memory>
#include <thread>
#include <csignal>
#include <utility>
#include <sstream>

#include <Eigen/Dense>
#include <ros/ros.h>

#include "yaml-cpp/yaml.h"
#include "map3d.h"
#include "warden.h"
#include "trajectory.h"
#include "trajectory_subscriber_node.h"

#include "quad_state.h"
#include "quad_state_publisher_node.h"

#include "mediation_layer.h"
#include "physics_simulator.h"
#include "wind_intensity.h"

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
  ros::init(argc, argv, "physics_simulator", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("/game_engine/");

  std::map<std::string, std::string> updated_trajectory_topics;
  if(false == nh.getParam("updated_trajectory_topics", updated_trajectory_topics)) {
    std::cerr << "Required parameter not found on server: updated_trajectory_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  std::map<std::string, std::string> quad_state_topics;
  if(false == nh.getParam("quad_state_topics", quad_state_topics)) {
    std::cerr << "Required parameter not found on server: quad_state_topics" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  auto trajectory_warden_sub  = std::make_shared<TrajectoryWardenSubscriber>();
  for(const auto& kv: updated_trajectory_topics) {
    const std::string& quad_name = kv.first;
    trajectory_warden_sub->Register(quad_name);
  }

    std::unordered_map<std::string, std::shared_ptr<TrajectorySubscriberNode>> trajectory_subscribers;
    for(const auto& kv: updated_trajectory_topics) {
        const std::string& quad_name = kv.first;
        const std::string& topic = kv.second;
        trajectory_subscribers[quad_name] =
                std::make_shared<TrajectorySubscriberNode>(
                        topic,
                        quad_name,
                        trajectory_warden_sub);
    }


    std::unordered_map<std::string, std::shared_ptr<QuadStatePublisherNode>> quad_state_publishers;
  for(const auto& kv: quad_state_topics) {
    const std::string& quad_name = kv.first;
    const std::string& topic = kv.second;
    quad_state_publishers[quad_name]
        = std::make_shared<QuadStatePublisherNode>(topic);
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

  // Obtain seed for balloon position change
  bool use_seed;
  if(false == nh.getParam("use_seed", use_seed)) {
    std::cerr << "Required parameter not found on server: use_seed" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  int seed;
  if (use_seed) {
    if(false == nh.getParam("seed", seed)) {
      std::cerr << "Required parameter not found on server: seed" << std::endl;
      std::exit(EXIT_FAILURE);
    }
  } else {
    seed = std::random_device{}();
  }

  // Configure wind intensity
  int wind_intensity_int = 0;
  if(false == nh.getParam("wind_intensity", wind_intensity_int)) {
    std::cerr << "Required parameter not found on server: wind_intensity" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  const WindIntensity wind_intensity =
    static_cast<WindIntensity>(wind_intensity_int);
  PhysicsSimulator::Options physics_simulator_options;
  
  switch(wind_intensity) {
  case WindIntensity::Zero:
    physics_simulator_options.sigma_u_x = 0.0001;
    physics_simulator_options.sigma_u_y = 0.0001;
    physics_simulator_options.sigma_u_z = 0.00005;
    break;
  case WindIntensity::Mild:
    physics_simulator_options.sigma_u_x = 0.1;
    physics_simulator_options.sigma_u_y = 0.1;
    physics_simulator_options.sigma_u_z = 0.05;
    break;
  case WindIntensity::Stiff:
    physics_simulator_options.sigma_u_x = 0.2;
    physics_simulator_options.sigma_u_y = 0.2;
    physics_simulator_options.sigma_u_z = 0.1;
    break;
  case WindIntensity::Intense:
    physics_simulator_options.sigma_u_x = 0.4;
    physics_simulator_options.sigma_u_y = 0.4;
    physics_simulator_options.sigma_u_z = 0.2;
    break;
  case WindIntensity::Ludicrous:
    physics_simulator_options.sigma_u_x = 0.8;
    physics_simulator_options.sigma_u_y = 0.8;
    physics_simulator_options.sigma_u_z = 0.4;
    break;
  default:
    std::cerr << "wind_intensity must be an integer on [" <<
      static_cast<int>(WindIntensity::Minimum) << "," <<
      static_cast<int>(WindIntensity::Maximum) << "]"  << std::endl;
    std::exit(EXIT_FAILURE);
  }
  
  physics_simulator_options.initial_quad_positions = initial_quad_positions;
  auto physics_simulator = std::make_shared<PhysicsSimulator>(physics_simulator_options);
  std::thread physics_simulator_thread(
      [&]() {
        physics_simulator->Run(trajectory_warden_sub, quad_state_publishers, seed);
      });

  // Kill program thread. This thread sleeps for a second and then checks if the
  // 'kill_program' variable has been set. If it has, it shuts ros down and
  // sends stop signals to any other threads that might be running.
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

        trajectory_warden_sub->Stop();
        physics_simulator->Stop();
      });

  // Spin for ros subscribers
  ros::spin();

  // Wait for program termination via ctl-c
  kill_thread.join();
  physics_simulator_thread.join();

  return EXIT_SUCCESS;
}
