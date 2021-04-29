
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

#include "quad_state.h"
#include "quad_state_guard.h"


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

        std::unordered_map <std::string, std::shared_ptr<Warden::Container>> map_;
        std::set <std::string> keys_;
        volatile std::atomic<bool> ok_{true};

    public:
        // Constructor
        Warden(){};

        // Add a key-value pair to the map
        TrajectoryCode Register(const std::string &key) {
            // If this key already exists, return false
            if (this->map_.end() != this->map_.find(key)) {
                std::cerr << "Warden::Register -- Key already exists." << std::endl;
                return TrajectoryCode::KeyAlreadyExists;
            }

            this->map_[key] = std::make_shared<Warden::Container>(T());
            // std::cout << "Quad: " << key << " successfully registered on the map." << std::endl;
            keys_.insert(key);
            return TrajectoryCode::Success;
        };

        // Copy the latest type T associated with a key
        TrajectoryCode Read(const std::string &key, T &type) {
            // If key does not exist, return false
            if (this->map_.end() == this->map_.find(key)) {
                std::cerr << "Warden::Read -- Key does not exist." << std::endl;
                return TrajectoryCode::KeyDoesNotExist;
            }
            std::shared_ptr <Container> &container = this->map_[key];

            { // Lock mutex for copy
                std::lock_guard <std::mutex> lock(container->modified_mtx_);
                type = container->type_;
            }

            return TrajectoryCode::Success;
        };

        // Await a change to the state associated with the key
        TrajectoryCode Await(const std::string &key, T &type) {
            if (this->map_.end() == this->map_.find(key)) {
                std::cerr << "Warden::Await -- Key does not exist." << std::endl;
                return TrajectoryCode::KeyDoesNotExist;
            }
            std::shared_ptr <Container> &container = this->map_[key];

            {// Lock mutex, wait cv, copy, set cv, release mutex
                std::unique_lock <std::mutex> lock(container->modified_mtx_);

                container->modified_cv_.wait(lock, [&] {
                    return (true == container->modified_) || (false == this->ok_);
                });

                // Termination
                if (false == this->ok_) {
                    return TrajectoryCode::ThreadStopped;
                }

                type = container->type_;
                container->modified_ = false;
                lock.unlock();
            }

            return TrajectoryCode::Success;
        };

        // Getter
        const std::set <std::string> &Keys() const {
            return this->keys_;
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
    class TrajectoryWardenIn : public Warden<Trajectory> {
    private:
        volatile std::atomic<bool> statusUpdated_{false};
        TrajectoryCode trajectoryStatus_{TrajectoryCode::Success};
        TrajectoryCode GetLastTrajectoryStatus(bool blocking);
    public:
        TrajectoryWardenIn(){};
        TrajectoryCode Write(const std::string& key,
                             const Trajectory& trajectory,
                             bool blocking = false);

        void SetTrajectoryStatus(TrajectoryCode status);
    };

    class TrajectoryWardenOut : public Warden<Trajectory> {
    private:
        volatile std::atomic<bool> statusUpdated_{false};
        TrajectoryCode trajectoryStatus_{TrajectoryCode::Success};
    public:
        TrajectoryWardenOut(){};
        TrajectoryCode Write(const std::string& key,
                             const Trajectory& trajectory,
                             std::unordered_map<std::string, std::shared_ptr<TrajectoryClientNode>> client,
                             bool blocking = false);

        void SetTrajectoryStatus(TrajectoryCode status);
    };

    class QuadStateWarden : public Warden<QuadState> {
    public:
        QuadStateWarden(){};
        TrajectoryCode Write(const std::string& key,
                             const QuadState& state);
    };
}
