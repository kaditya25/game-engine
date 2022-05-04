#pragma once

#include <memory>
#include <vector>
#include <chrono>
#include <iostream>

#include "graph.h"
#include "timer.h"
#include "path_info.h"

namespace game_engine {

  struct AStar3D {
    // Students will implement this function
    PathInfo Run(const Graph3D& graph, 
                 const std::shared_ptr<Node3D> start_ptr, 
                 const std::shared_ptr<Node3D> end_ptr);
  };
}
