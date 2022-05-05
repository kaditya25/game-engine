#include "warden.h"

namespace game_engine {
//============================
//     TrajectoryWardenIn
//============================
TrajectoryCode TrajectoryWardenServer::GetLastTrajectoryStatus(
    const std::string& key) {
  // Checks to see if the status hasn't been updated yet.
  // Wait until the statusUpdated_ marker returns true and return the status.
  while (this->statusUpdated_[key] == false) {
    continue;
  }
  this->statusUpdated_[key] = false;
  return trajectoryStatus_[key];
};

TrajectoryCode TrajectoryWardenServer::Write(const std::string& key,
                                             const Trajectory& trajectory) {
  // If key does not exist, return false
  if (this->map_.end() == this->map_.find(key)) {
    std::cerr << "TrajectoryWardenServer::Write -- Key does not exist."
              << std::endl;
    TrajectoryCode tc;
    tc.code = MediationLayerCode::KeyDoesNotExist;
    return tc;
  }

  std::shared_ptr<Warden<Trajectory>::Container>& container = this->map_[key];

  {  // Lock mutex for modification
    std::lock_guard<std::mutex> lock(container->modified_mtx_);
    container->type_ = trajectory;
    container->modified_ = true;
    container->modified_cv_.notify_all();
  }
  // The GetLastTrajectoryStatus grabs the most recent status updated by the ML.
  TrajectoryCode status = GetLastTrajectoryStatus(key);
  return status;
};

void TrajectoryWardenServer::SetTrajectoryStatus(
    const std::string& key, TrajectoryCode trajectory_status) {
  // Set the two variables to true if the status was updated and the value of
  // the status
  this->trajectoryStatus_[key] = trajectory_status;
  this->statusUpdated_[key] = true;
};

//==============================
//     TrajectoryWardenClient
//==============================

TrajectoryCode TrajectoryWardenClient::Write(
    const std::string& key, const Trajectory& trajectory,
    std::unordered_map<std::string, std::shared_ptr<TrajectoryClientNode>>
        client) {
  // If key does not exist, return false
  if (this->map_.end() == this->map_.find(key)) {
    std::cerr << "TrajectoryWardenClient::Write -- Key does not exist."
              << std::endl;
    TrajectoryCode tc;
    tc.code = MediationLayerCode::KeyDoesNotExist;
    return tc;
  }

  std::shared_ptr<Warden<Trajectory>::Container>& container = this->map_[key];

  {  // Lock mutex for modification
    std::lock_guard<std::mutex> lock(container->modified_mtx_);
    container->type_ = trajectory;
    container->modified_ = true;
    container->modified_cv_.notify_all();
  }

  // The TrajectoryWardenOut gets sends out a call from the client to the server
  // and gets the status back
  TrajectoryCode status = client[key]->Request(trajectory);
  // Update the status so the In warden is released from it's block.
  SetTrajectoryStatus(key, status);

  return status;
};

void TrajectoryWardenClient::SetTrajectoryStatus(
    const std::string& key, TrajectoryCode trajectory_status) {
  this->trajectoryStatus_[key] = trajectory_status;
  this->statusUpdated_[key] = true;
};

//====================================
//     TrajectoryWardenSubscriber
//====================================
MediationLayerCode TrajectoryWardenSubscriber::Write(
    const std::string& key, const Trajectory& trajectory) {
  // If key does not exist, return false
  if (this->map_.end() == this->map_.find(key)) {
    std::cerr << "TrajectoryWardenSubscriber::Write -- Key does not exist."
              << std::endl;
    return MediationLayerCode::KeyDoesNotExist;
  }

  std::shared_ptr<Warden<Trajectory>::Container>& container = this->map_[key];

  {  // Lock mutex for modification
    std::lock_guard<std::mutex> lock(container->modified_mtx_);
    container->type_ = trajectory;
    container->modified_ = true;
    container->modified_cv_.notify_all();
  }

  return MediationLayerCode::Success;
};

//=====================================
//     TrajectoryWardenPublisher
//=====================================

MediationLayerCode TrajectoryWardenPublisher::Write(
    const std::string& key, const Trajectory& trajectory,
    std::shared_ptr<TrajectoryPublisherNode> publisher) {
  // If key does not exist, return false
  if (this->map_.end() == this->map_.find(key)) {
    std::cerr << "TrajectoryWardenPublisher::Write -- Key does not exist."
              << std::endl;
    return MediationLayerCode::KeyDoesNotExist;
  }

  std::shared_ptr<Warden<Trajectory>::Container>& container = this->map_[key];

  {  // Lock mutex for modification
    std::lock_guard<std::mutex> lock(container->modified_mtx_);
    container->type_ = trajectory;
    container->modified_ = true;
    container->modified_cv_.notify_all();
  }

  // publish trajectory
  publisher->Publish(trajectory);

  return MediationLayerCode::Success;
};

//============================
//     QuadStateWarden
//============================

MediationLayerCode QuadStateWarden::Write(const std::string& key,
                                          const QuadState& state) {
  // If key does not exist, return false
  if (this->map_.end() == this->map_.find(key)) {
    std::cerr << "QuadStateWarden::Write -- Key does not exist." << std::endl;
    return MediationLayerCode::KeyDoesNotExist;
  }
  std::shared_ptr<Warden<QuadState>::Container>& container = this->map_[key];

  {  // Lock mutex for modification
    std::lock_guard<std::mutex> lock(container->modified_mtx_);
    container->type_ = state;
    container->modified_ = true;
    container->modified_cv_.notify_all();
  }

  return MediationLayerCode::Success;
};
}  // namespace game_engine
