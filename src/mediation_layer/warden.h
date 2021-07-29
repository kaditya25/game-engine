#pragma once

#include <string>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <set>
#include <iostream>
#include <condition_variable>
#include <atomic>

#include "trajectory_code.h"
#include "trajectory.h"
#include "trajectory_client.h"
#include "trajectory_publisher_node.h"

#include "quad_state.h"

namespace game_engine {
    // Warden encapsulates state data and provides thread-safe read, write,
    // and await-modification access.
    template<class T>
    class Warden {
    protected:
        // Wraps a type T with local mutexes and condition variables that
        // ensure thread-safe access
        struct Container {
            std::mutex modified_mtx_;
            bool modified_{false};
            std::condition_variable modified_cv_;
            T type_;

            Container(const T &type) : type_(type) {}
        };

        std::unordered_map<std::string, std::shared_ptr<Warden::Container>> map_;
        std::set<std::string> keys_;
        volatile std::atomic<bool> ok_{true};

    public:
        // Constructor
        Warden(){};

        // Add a key-value pair to the map
        MediationLayerCode Register(const std::string &key) {
            // If this key already exists, return false
            if (this->map_.end() != this->map_.find(key)) {
                std::cerr << "Warden::Register -- Key already exists." << std::endl;
                return MediationLayerCode::KeyAlreadyExists;
            }

            this->map_[key] = std::make_shared<Warden::Container>(T());
            keys_.insert(key);
            return MediationLayerCode::Success;
        };

        // Copy the latest type T associated with a key
        MediationLayerCode Read(const std::string &key, T &type) {
            // If key does not exist, return false
            if (this->map_.end() == this->map_.find(key)) {
                std::cerr << "Warden::Read -- Key does not exist." << std::endl;
                return MediationLayerCode::KeyDoesNotExist;
            }
            std::shared_ptr <Container> &container = this->map_[key];

            { // Lock mutex for copy
                std::lock_guard <std::mutex> lock(container->modified_mtx_);
                type = container->type_;
            }

            return MediationLayerCode::Success;
        };

        // Await a change to the state associated with the key
        MediationLayerCode Await(const std::string &key, T &type) {
            if (this->map_.end() == this->map_.find(key)) {
                std::cerr << "Warden::Await -- Key does not exist." << std::endl;
                return MediationLayerCode::KeyDoesNotExist;
            }
            std::shared_ptr <Container> &container = this->map_[key];

            {// Lock mutex, wait cv, copy, set cv, release mutex
                std::unique_lock <std::mutex> lock(container->modified_mtx_);

                container->modified_cv_.wait(lock, [&] {
                    return (true == container->modified_) || (false == this->ok_);
                });

                // Termination
                if (false == this->ok_) {
                    return MediationLayerCode::ThreadStopped;
                }

                type = container->type_;
                container->modified_ = false;
                lock.unlock();
            }

            return MediationLayerCode::Success;
        };

        // Getter
        const std::set <std::string> &Keys() const {
            return this->keys_;
        };

        // Check if the container is modified
        bool ModifiedStatus(const std::string &key){
            std::shared_ptr <Container> &container = this->map_[key];
            return container->modified_;
        };

        // Break all condition variable wait statements
        void Stop() {
            this->ok_ = false;

            // Notify all CV to check conditions
            for (const auto &kv: this->map_) {
                std::lock_guard <std::mutex> lock(kv.second->modified_mtx_);
                kv.second->modified_cv_.notify_all();
            }
        };
    };

    //============================
    //     INHERITED CLASSES
    //============================
    class TrajectoryWardenServer : public Warden<Trajectory> {
    private:
        std::unordered_map<std::string, volatile std::atomic<bool>> statusUpdated_;
        std::unordered_map<std::string, TrajectoryCode> trajectoryStatus_;
        TrajectoryCode GetLastTrajectoryStatus(const std::string& key);
    public:
        TrajectoryWardenServer(){
//          const std::set<std::string> state_keys = Keys();
          for(const std::string& key: Keys()) {
            statusUpdated_[key] = false;
          }
        };
        TrajectoryCode Write(const std::string& key,
                             const Trajectory& trajectory);

        void SetTrajectoryStatus(const std::string& key,
                                 TrajectoryCode status);
    };

    class TrajectoryWardenClient : public Warden<Trajectory> {
    private:
        std::unordered_map<std::string, volatile std::atomic<bool>> statusUpdated_;
        std::unordered_map<std::string, TrajectoryCode> trajectoryStatus_;
    public:
        TrajectoryWardenClient(){
          for(const std::string& key: Keys()) {
            statusUpdated_[key] = false;
          }
        };
        TrajectoryCode Write(const std::string& key,
                             const Trajectory& trajectory,
                             std::unordered_map<std::string, std::shared_ptr<TrajectoryClientNode>> client);

        void SetTrajectoryStatus(const std::string& key,
                                 TrajectoryCode status);
    };

    class TrajectoryWardenSubscriber : public Warden<Trajectory> {
    public:
        TrajectoryWardenSubscriber(){};
        MediationLayerCode Write(const std::string& key,
                                 const Trajectory& trajectory);
    };

    class TrajectoryWardenPublisher : public Warden<Trajectory> {
    public:
        TrajectoryWardenPublisher(){};
        MediationLayerCode Write(const std::string& key,
                                 const Trajectory& trajectory,
                                 std::shared_ptr<TrajectoryPublisherNode> publisher);
    };

    class QuadStateWarden : public Warden<QuadState> {
    public:
        QuadStateWarden(){};
        MediationLayerCode Write(const std::string& key,
                                 const QuadState& state);
    };
}
