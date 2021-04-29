#include "warden.h"

namespace game_engine {
    //============================
    //     TrajectoryWardenIn
    //============================
    TrajectoryCode TrajectoryWardenIn::GetLastTrajectoryStatus(bool blocking) {
        // Checks to see if the status hasn't been updated yet and a blocking call is wanted.
        // If a blocking call is wanted, then wait until the statusUpdated_ marker returns true.
        // If a blocking call is not wanted, just return the most recently updated status.
        while (this->statusUpdated_ == false && blocking == true) {
            continue;
        }
        this->statusUpdated_ = false;
        return static_cast<TrajectoryCode>(trajectoryStatus_);
    };

    TrajectoryCode TrajectoryWardenIn::Write(const std::string& key,
                                             const Trajectory& trajectory,
                                             bool blocking) {
        // If key does not exist, return false
        if (this->map_.end() == this->map_.find(key)) {
            std::cerr << "TrajectoryWardenIn::Write -- Key does not exist." << std::endl;
            return TrajectoryCode::KeyDoesNotExist;
        }

        std::shared_ptr <Warden<Trajectory>::Container> &container = this->map_[key];

        { // Lock mutex for modification
            std::lock_guard <std::mutex> lock(container->modified_mtx_);
            container->type_ = trajectory;
            container->modified_ = true;
            container->modified_cv_.notify_all();
        }
        // The TrajectoryWardenIn is a "read" warden that changes its behavior based on whether or not
        // it is a blocking call. The GetLastTrajectoryStatus grabs the most recent status updated by
        // the ML.
        TrajectoryCode status = GetLastTrajectoryStatus(blocking);
        return status;
    };

    void TrajectoryWardenIn::SetTrajectoryStatus(TrajectoryCode status) {
        // Set the two variables to true if the status was updated and the value of the status
        this->trajectoryStatus_ = status;
        this->statusUpdated_ = true;
    };

    //============================
    //     TrajectoryWardenOut
    //============================

    TrajectoryCode TrajectoryWardenOut::Write(const std::string& key,
                                              const Trajectory& trajectory,
                                              std::unordered_map<std::string, std::shared_ptr<TrajectoryClientNode>> client,
                                              bool blocking) {
        // If key does not exist, return false
        if (this->map_.end() == this->map_.find(key)) {
            std::cerr << "TrajectoryWardenOut::Write -- Key does not exist." << std::endl;
            return TrajectoryCode::KeyDoesNotExist;
        }

        std::shared_ptr <Warden<Trajectory>::Container> &container = this->map_[key];

        { // Lock mutex for modification
            std::lock_guard <std::mutex> lock(container->modified_mtx_);
            container->type_ = trajectory;
            container->modified_ = true;
            container->modified_cv_.notify_all();
        }

        // The TrajectoryWardenOut gets sends out a call from the client to the server and gets the status back
        TrajectoryCode status = client[key]->Request(trajectory);

        // If blocking is true, update the status so the In warden is released from it's block.
        if (blocking == true) {
            // Update the status
            SetTrajectoryStatus(status);
        }
        return status;
    };

    void TrajectoryWardenOut::SetTrajectoryStatus(TrajectoryCode status) {
        this->trajectoryStatus_ = status;
        this->statusUpdated_ = true;
    };

    //====================================
    //     TrajectoryWardenIn_PubSub
    //====================================
    TrajectoryCode TrajectoryWardenIn_PubSub::Write(const std::string& key,
                                                    const Trajectory& trajectory) {
        // If key does not exist, return false
        if (this->map_.end() == this->map_.find(key)) {
            std::cerr << "TrajectoryWardenIn::Write -- Key does not exist." << std::endl;
            return TrajectoryCode::KeyDoesNotExist;
        }

        std::shared_ptr <Warden<Trajectory>::Container> &container = this->map_[key];

        { // Lock mutex for modification
            std::lock_guard <std::mutex> lock(container->modified_mtx_);
            container->type_ = trajectory;
            container->modified_ = true;
            container->modified_cv_.notify_all();
        }

        return TrajectoryCode::Success;
    };

    //=====================================
    //     TrajectoryWardenOut_PubSub
    //=====================================

    TrajectoryCode TrajectoryWardenOut_PubSub::Write(const std::string& key,
                                                     const Trajectory& trajectory,
                                                     std::unordered_map<std::string, std::shared_ptr<TrajectoryPublisherNode>> publisher) {
        // If key does not exist, return false
        if (this->map_.end() == this->map_.find(key)) {
            std::cerr << "TrajectoryWardenOut_PubSub::Write -- Key does not exist." << std::endl;
            return TrajectoryCode::KeyDoesNotExist;
        }

        std::shared_ptr <Warden<Trajectory>::Container> &container = this->map_[key];

        { // Lock mutex for modification
            std::lock_guard <std::mutex> lock(container->modified_mtx_);
            container->type_ = trajectory;
            container->modified_ = true;
            container->modified_cv_.notify_all();
        }

        // The TrajectoryWardenOut publishes
        publisher[key]->Publish(trajectory);

        return TrajectoryCode::Success;
    };

    //============================
    //     QuadStateWarden
    //============================

    TrajectoryCode QuadStateWarden::Write(const std::string& key,
                                          const QuadState& state) {
        // If key does not exist, return false
        if (this->map_.end() == this->map_.find(key)) {
            std::cerr << "QuadStateWarden::Write -- Key does not exist." << std::endl;
            return TrajectoryCode::KeyDoesNotExist;
        }
        std::shared_ptr <Warden<QuadState>::Container> &container = this->map_[key];

        { // Lock mutex for modification
            std::lock_guard <std::mutex> lock(container->modified_mtx_);
            container->type_ = state;
            container->modified_ = true;
            container->modified_cv_.notify_all();
        }

        return TrajectoryCode::Success;
    };
}
