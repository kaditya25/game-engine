#ifndef SAFETY_MONITOR_STATUS_H
#define SAFETY_MONITOR_STATUS_H

#include <atomic>
#include <string>
#include <mutex>
#include <unordered_map>
#include <stdexcept>
#include <iostream>
#include "quad_safety_status.h"

namespace game_engine {

    class SafetyMonitorStatus {
    private:
        // Mutex to manage multi-threaded access
        mutable std::mutex mtx_;
        // Map from quad name to whether or not an infraction has occurred
        std::unordered_map <std::string, QuadSafetyStatus> safety_map_;
    public:
        SafetyMonitorStatus() {}

        void Register(const std::string &quad_name);

        QuadSafetyStatus Read(const std::string &quad_name) const;

        void Write(const std::string &quad_name, const QuadSafetyStatus current_safety_level);
    };
}

#endif