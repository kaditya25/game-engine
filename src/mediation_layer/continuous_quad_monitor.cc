#include "continuous_quad_monitor.h"

#include <Eigen/Core>
#include <iostream>

namespace game_engine {

    MediationLayerCode ContinuousQuadMonitor::
    SimulateForward(const Trajectory& trajectory,
                    const Map3D& map,
                    const std::shared_ptr<QuadStateWarden> quad_state_warden,
                    const std::string& quad_name,
                    const std::vector<std::string>& other_quad_names) const {


      return MediationLayerCode::Success;
    }
}