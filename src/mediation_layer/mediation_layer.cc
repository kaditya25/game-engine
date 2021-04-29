#include <thread>
#include <chrono>

#include "mediation_layer.h"
#include "trajectory_vetter.h"

namespace game_engine {
    void MediationLayer::TransferData(
       const std::string& key,
       const Map3D& map,
       std::shared_ptr<TrajectoryWardenIn> trajectory_warden_in,
       std::shared_ptr<TrajectoryWardenOut_PubSub> trajectory_warden_out,
       std::shared_ptr<QuadStateWarden> quad_state_warden,
       std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
       std::unordered_map<std::string, std::shared_ptr<TrajectoryPublisherNode>> trajectory_publishers) {

      TrajectoryVetter trajectory_vetter;
      while(true == this->ok_) {
        // Determine if quad has violated state constraints. If it has, freeze
        // it in place
        if(true == quad_state_watchdog_status->Read(key)) {
          std::cout << key << " has flown too close to obstacle or boundary. Freezing." << std::endl;

          QuadState current_quad_state;
          quad_state_warden->Read(key, current_quad_state);

          static const Eigen::Vector3d freeze_quad_position = current_quad_state.Position();

          const double current_time =
            std::chrono::duration_cast<std::chrono::duration<double>>
            (std::chrono::system_clock::now().time_since_epoch()).count();

          TrajectoryVector3D freeze_trajectory_vector;
          for(size_t idx = 0; idx < 100; ++idx) {
            freeze_trajectory_vector.push_back((Eigen::Matrix<double, 11, 1>() <<
                                                freeze_quad_position.x(),
                                                freeze_quad_position.y(),
                                                freeze_quad_position.z(),
                                                0,0,0,
                                                0,0,0,
                                                0,
                                                current_time + idx * 0.01
                                                ).finished()
                                                );
          }
          const Trajectory freeze_trajectory(freeze_trajectory_vector);

          trajectory_warden_out->Write(key, freeze_trajectory_vector, trajectory_publishers);
          continue;
        }

        // Determine if trajectory has violated constraints
        Trajectory trajectory;
        trajectory_warden_in->Await(key, trajectory);
        TrajectoryCode trajectoryCode =
          trajectory_vetter.Vet(trajectory, map, quad_state_warden, key);
        trajectory_warden_in->SetTrajectoryStatus(trajectoryCode);
        if (trajectoryCode != TrajectoryCode::Success) {
          std::cerr << "Trajectory did not pass vetting: rejected with code "
                    << static_cast<unsigned int>(trajectoryCode) <<
                    "." << std::endl;
          continue;
        }

        //trajectory_warden_out->SetTrajectoryStatus(trajectoryCode);
        trajectory_warden_out->Write(key, trajectory, trajectory_publishers);
    }
  }

  void MediationLayer::
  Run(const Map3D& map,
      std::shared_ptr<TrajectoryWardenIn> trajectory_warden_in,
      std::shared_ptr<TrajectoryWardenOut_PubSub> trajectory_warden_out,
      std::shared_ptr<QuadStateWarden> quad_state_warden,
      std::shared_ptr<QuadStateWatchdogStatus> quad_state_watchdog_status,
      std::unordered_map<std::string, std::shared_ptr<TrajectoryPublisherNode>> trajectory_publishers) {

    // Local thread pool
    std::vector<std::thread> thread_pool;

    // Get all registered trajectories
    const std::set<std::string> state_keys = quad_state_warden->Keys();

    // Assign a thread to await changes in each trajectory
    for(const std::string& key: state_keys) {
      thread_pool.push_back(std::move(std::thread([&](){
          TransferData(key, map,
                       trajectory_warden_in,
                       trajectory_warden_out,
                       quad_state_warden,
                       quad_state_watchdog_status,
                       trajectory_publishers);})));
    }

    // Wait for this thread to receive a stop command
    std::thread kill_thread([&, this]() {
                              while(true) {
                                if(false == ok_) {
                                  break;
                                } else {
                                  std::this_thread::
                                    sleep_for(std::chrono::milliseconds(1000));
                                }
                              }
                            });

    kill_thread.join();

    // Wait for thread pool to terminate
    for(std::thread& t: thread_pool) {
      t.join();
    }
  }

  void MediationLayer::Stop() {
    ok_ = false;
  }
}
