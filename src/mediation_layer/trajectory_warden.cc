#include "trajectory_warden.h"

namespace game_engine {
    TrajectoryCode TrajectoryWarden::Register(const std::string& key) {
        // If this key already exists, return false
        if(this->map_.end() != this->map_.find(key)) {
            std::cerr << "TrajectoryWarden::Register -- Key already exists." << std::endl;
            return TrajectoryCode::KeyAlreadyExists;
        }

        this->map_[key] = std::make_shared<TrajectoryContainer>(Trajectory());
        std::cout << "Key: " << key << std::endl;
        keys_.insert(key);
        return TrajectoryCode::Success;
    }

    TrajectoryCode TrajectoryWardenIn::Write(const std::string& key, const Trajectory& trajectory, bool blocking) {
        // If key does not exist, return false
        if(this->map_.end() == this->map_.find(key)) {
            std::cerr << "TrajectoryWardenIn::Write -- Key does not exist." << std::endl;
            return TrajectoryCode::KeyDoesNotExist;
        }

        std::shared_ptr<TrajectoryContainer>& container = this->map_[key];

        { // Lock mutex for modification
            std::lock_guard <std::mutex> lock(container->modified_mtx_);
            container->trajectory_ = trajectory;
            container->modified_ = true;
            container->modified_cv_.notify_all();
        }

        TrajectoryCode status = GetLastTrajectoryStatus(blocking);
        return status;
    }

    TrajectoryCode TrajectoryWardenOut::Write(const std::string& key,
                                              const Trajectory& trajectory,
                                              std::unordered_map<std::string, std::shared_ptr<TrajectoryClientNode>> client,
                                              bool blocking) {
        // If key does not exist, return false
        if(this->map_.end() == this->map_.find(key)) {
            std::cerr << "TrajectoryWardenOut::Write -- Key does not exist." << std::endl;
            return TrajectoryCode::KeyDoesNotExist;
        }

        std::shared_ptr<TrajectoryContainer>& container = this->map_[key];

        { // Lock mutex for modification
            std::lock_guard<std::mutex> lock(container->modified_mtx_);
            container->trajectory_ = trajectory;
            container->modified_ = true;
            container->modified_cv_.notify_all();
        }

        // Call client
        TrajectoryCode status = client[key]->Request(trajectory);

        if(blocking == true) {
            // Update the status
            SetTrajectoryStatus(status);
        }
        return status;
    }

    TrajectoryCode TrajectoryWarden::Read(const std::string& key, Trajectory& trajectory) {
        // If key does not exist, return false
        if(this->map_.end() == this->map_.find(key)) {
            std::cerr << "TrajectoryWarden::Read -- Key does not exist." << std::endl;
            return TrajectoryCode::KeyDoesNotExist;
        }
        std::shared_ptr<TrajectoryContainer>& container = this->map_[key];

        { // Lock mutex for copy
            std::lock_guard<std::mutex> lock(container->modified_mtx_);
            trajectory = container->trajectory_;
        }

        return TrajectoryCode::Success;
    }

    TrajectoryCode TrajectoryWarden::Await(const std::string& key,
                                           Trajectory& trajectory,
                                           std::unordered_map<std::string, std::shared_ptr<TrajectoryClientNode>> client) {
        if(this->map_.end() == this->map_.find(key)) {
            std::cerr << "TrajectoryWarden::Await -- Key does not exist." << std::endl;
            return TrajectoryCode::KeyDoesNotExist;
        }
        std::shared_ptr<TrajectoryContainer>& container = this->map_[key];

        {// Lock mutex, wait cv, copy, set cv, release mutex
            std::unique_lock <std::mutex> lock(container->modified_mtx_);

            container->modified_cv_.wait(lock, [&] {
                return (true == container->modified_) || (false == this->ok_); });

            // Termination
            if (false == this->ok_) {
                return TrajectoryCode::ThreadStopped;
            }

            trajectory = container->trajectory_;
            container->modified_ = false;
            lock.unlock();
        }

        return TrajectoryCode::Success;
    }

    const std::set<std::string>& TrajectoryWarden::Keys() const {
        return this->keys_;
    }

    void TrajectoryWarden::SetTrajectoryStatus(TrajectoryCode status) {
        this->trajectoryStatus_ = status;
        this->statusUpdated_ = true;
    }

    TrajectoryCode TrajectoryWardenIn::GetLastTrajectoryStatus(bool blocking) {
        // checks to see if the status hasn't been updated and we want a blocking call
        while (this->statusUpdated_ == false && blocking == true) {
            continue;
        }

        this->statusUpdated_ = false;
        return static_cast<TrajectoryCode>(trajectoryStatus_);
    }

    void TrajectoryWarden::Stop() {
        this->ok_ = false;

        // Notify all CV to check conditions
        for(const auto& kv: this->map_) {
            std::lock_guard<std::mutex> lock(kv.second->modified_mtx_);
            kv.second->modified_cv_.notify_all();
        }
    }
}
