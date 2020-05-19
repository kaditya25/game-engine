#include "view_manager.h"

namespace game_engine {
  void ViewManager::Run(const QuadViewOptions quad_view_options,
                        const BalloonViewOptions balloon_view_options,
                        const GoalViewOptions goal_view_options,
                        const EnvironmentViewOptions environment_view_options,
                        const TrajectoryViewOptions trajectory_view_options) {

    std::thread quad_publisher_thread(
        [&]() {
          RunQuadPublisher(quad_view_options);
        });

    std::thread balloon_publisher_thread(
        [&]() {
          RunBalloonPublisher(balloon_view_options);
        });

    std::thread goal_publisher_thread(
        [&]() {
          RunGoalPublisher(goal_view_options);
        });

    std::thread environment_publisher_thread(
        [&]() {
          RunEnvironmentPublisher(environment_view_options);
        });

    std::thread trajectory_publisher_thread(
        [&]() {
          RunTrajectoryPublisher(trajectory_view_options);
        });

    quad_publisher_thread.join();
    balloon_publisher_thread.join();
    goal_publisher_thread.join();
    environment_publisher_thread.join();
    trajectory_publisher_thread.join();
  }

  void ViewManager::RunQuadPublisher(
      const QuadViewOptions quad_view_options) {

    // Setup
    std::vector<QuadView> quad_views;

    for(const auto p: quad_view_options.quads) {
      if(p.first == "red") {
        QuadView::Options view_options;
        view_options.mesh_resource = quad_view_options.quad_mesh_file_path;
        // UT blue-grey
        view_options.r = 0.2f;
        view_options.g = 0.247f;
        view_options.b = 0.28f;
        quad_views.emplace_back(p.second, view_options);
      }
      else if(p.first == "blue") {
        QuadView::Options view_options;
        view_options.mesh_resource = quad_view_options.quad_mesh_file_path;
        // burnt orange
        view_options.r = 0.75f;
        view_options.g = 0.34;
        view_options.b = 0.0f;
        quad_views.emplace_back(p.second, view_options);
      }
    }

    auto quads_publisher = std::make_shared<MarkerPublisherNode>("quads");

    // Main loop
    // 50 Hz. The quads move quickly and update often
    while(this->ok_) {
      for(const auto& view: quad_views) {
        for(const visualization_msgs::Marker& marker: view.Markers()) {
          quads_publisher->Publish(marker);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void ViewManager::RunBalloonPublisher(
      const BalloonViewOptions balloon_view_options) {

    // Setup
    std::vector<BalloonView> balloon_views;

    for(auto p: balloon_view_options.balloons) {
      if(p.first == "red") {
        BalloonView::Options view_options;
        view_options.mesh_resource = balloon_view_options.balloon_mesh_file_path;
        view_options.r = 1.0f;
        view_options.g = 0.0f;
        view_options.b = 0.0f;
        balloon_views.emplace_back(p.second, view_options);
      }
      else if(p.first == "blue") {
        BalloonView::Options view_options;
        view_options.mesh_resource = balloon_view_options.balloon_mesh_file_path;
        view_options.r = 0.0f;
        view_options.g = 0.0f;
        view_options.b = 1.0f;
        balloon_views.emplace_back(p.second, view_options);
      }
    }

    auto balloons_publisher = std::make_shared<MarkerPublisherNode>("balloons");

    // Code to get balloons to "pop" in visualizer
    // Balloon Status
    ros::NodeHandle nh("/game_engine/");
    std::map<std::string, std::string> balloon_status_topics;
    if(false == nh.getParam("balloon_status_topics", balloon_status_topics)) {
      std::cerr << "Required parameter not found on server: balloon_status_topics" << std::endl;
      std::exit(EXIT_FAILURE);
    }

    auto red_balloon_status = std::make_shared<BalloonStatus>();
    auto blue_balloon_status = std::make_shared<BalloonStatus>();

    auto red_balloon_status_subscriber_node 
      = std::make_shared<BalloonStatusSubscriberNode>(balloon_status_topics["red"], red_balloon_status);
    auto blue_balloon_status_subscriber_node 
      = std::make_shared<BalloonStatusSubscriberNode>(balloon_status_topics["blue"], blue_balloon_status);


    // Main loop
    // 50 Hz. 
    while(this->ok_) {
      for(auto& view: balloon_views) {
        // check for balloon motion
        // if balloon view position is not approx equal to balloon status position,
        // set balloon view position to balloon status position
        if (view.options_.r == 1.0f){ // red balloon
          if (!view.balloon_position_.isApprox(red_balloon_status_subscriber_node->balloon_status_->position)) {
            view.balloon_position_ = red_balloon_status_subscriber_node->balloon_status_->position;
          }
        } else if (view.options_.b == 1.0f) { // blue balloon{
          if (!view.balloon_position_.isApprox(blue_balloon_status_subscriber_node->balloon_status_->position)) {
            view.balloon_position_ = blue_balloon_status_subscriber_node->balloon_status_->position;
          }
        }
        
        for(visualization_msgs::Marker& marker: view.Markers()) {
          if (marker.color.r == 1.0f) {
            if (red_balloon_status_subscriber_node->balloon_status_->popped){
              // "pop" red balloon
              marker.action = visualization_msgs::Marker::DELETE;
              balloons_publisher->Publish(marker);
            } else {
              marker.action = visualization_msgs::Marker::ADD;
              balloons_publisher->Publish(marker);
            }
          } else if (marker.color.b == 1.0f) {
            if (blue_balloon_status_subscriber_node->balloon_status_->popped){
              // "pop" blue balloon
              marker.action = visualization_msgs::Marker::DELETE;
              balloons_publisher->Publish(marker);
            } else {
              marker.action = visualization_msgs::Marker::ADD;
              balloons_publisher->Publish(marker);
            }
          } else {
            balloons_publisher->Publish(marker);
          }
          
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void ViewManager::RunGoalPublisher(
      const GoalViewOptions goal_view_options) {

    // Setup
    std::vector<GoalView> goal_views;

    for(auto p: goal_view_options.goals) {
      if(p.first == "home") {
        GoalView::Options view_options;
        view_options.mesh_resource = goal_view_options.goal_mesh_file_path;
        view_options.r = 1.0f;
        view_options.g = 0.0f;
        view_options.b = 1.0f;
        goal_views.emplace_back(p.second, view_options);
      }
    }

    auto goal_publisher = std::make_shared<MarkerPublisherNode>("goal");

    ros::NodeHandle nh("/game_engine/");
    std::map<std::string, std::string> goal_status_topics;
    if(false == nh.getParam("goal_status_topics", goal_status_topics)) {
      std::cerr << "Required parameter not found on server: goal_status_topics" << std::endl;
      std::exit(EXIT_FAILURE);
    }

    auto goal_status = std::make_shared<GoalStatus>();

    auto goal_status_subscriber_node 
      = std::make_shared<GoalStatusSubscriberNode>(goal_status_topics["home"], goal_status);

    // Main loop
    // 50 Hz. 
    while(this->ok_) {
      for(auto& view: goal_views) {
        for(visualization_msgs::Marker& marker: view.Markers()) {
          bool active = goal_status_subscriber_node->goal_status_->active;
          bool reached = goal_status_subscriber_node->goal_status_->reached;
          if (active){
            marker.color.a = 1.0;
            marker.action = visualization_msgs::Marker::ADD;
            goal_publisher->Publish(marker);
          } else if (reached && !active) {
            marker.color.a = 0.3;
            //marker.action = visualization_msgs::Marker::ADD;
            marker.action = visualization_msgs::Marker::DELETE;
            goal_publisher->Publish(marker);
          } else {
            marker.action = visualization_msgs::Marker::DELETE;
            goal_publisher->Publish(marker);
          }
          
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void ViewManager::RunEnvironmentPublisher(
      const EnvironmentViewOptions environment_view_options) {

    // Setup
    std::vector<Plane3DView> plane_views;
    for(const Plane3D& wall: environment_view_options.map.Walls()) {  
      plane_views.emplace_back(
          wall, 
          environment_view_options.wall_view_options);
    }
    plane_views.emplace_back(
        environment_view_options.map.Ground(), 
        environment_view_options.ground_view_options);

    std::vector<PolyhedronView> obstacle_views;
    for(const Polyhedron& obstacle: environment_view_options.map.Obstacles()) {
      obstacle_views.emplace_back(
          obstacle, 
          environment_view_options.obstacle_view_options);
    }

    auto environment_publisher = std::make_shared<MarkerPublisherNode>("environment");

    // Main loop
    // 2 Hz. Environment does not change often
    while(this->ok_) {
      for(const Plane3DView& view: plane_views) {
        for(const visualization_msgs::Marker& marker: view.Markers()) {
          environment_publisher->Publish(marker);
        }
      }

      for(const PolyhedronView& obstacle_view: obstacle_views) {
        for(const visualization_msgs::Marker& marker: obstacle_view.Markers()) {
          environment_publisher->Publish(marker);
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  }

  void ViewManager::
  RunTrajectoryPublisher(const TrajectoryViewOptions trajectory_view_options) {
    
    // Main loop @ 5 Hz to keep up with quickly-updating trajectories
    while(this->ok_) {
      for(const auto& tr : trajectory_view_options.trajectories) {
        tr.second->Publish();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }

  void ViewManager::Stop() {
    this->ok_ = false;
  }
}

