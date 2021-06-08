#pragma once

#include <memory>
#include <string>

#include "trajectory.h"
#include "map3d.h"
#include "warden.h"
#include "trajectory_code.h"

namespace game_engine {
    class ContinuousQuadMonitor {
    public:
        struct Options {
            double simulation_forward_time = 5; //seconds

            Options() {}
        };

        ContinuousQuadMonitor(const Options& options = Options())
            : options_(options) {}

        // Determines if a trajectory meets the trajectory requirements laid out
        // in the documentation
        MediationLayerCode SimulateForward(
            const Trajectory& trajectory,
            const Map3D& map,
            const std::shared_ptr<QuadStateWarden> quad_state_warden,
            const std::string& quad_name,
            const std::vector<std::string>& other_quad_names
        ) const;

    private:
        Options options_;

    };
}
