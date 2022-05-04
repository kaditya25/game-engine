#include <cstdlib>
#include <vector>

#include "a_star3d.h"
// #include "gnuplot-iostream.h"
#include "occupancy_grid3d.h"
#include "path_info.h"
#include "polynomial_sampler.h"
#include "polynomial_solver.h"
// #include "gui2d.h"
#include <fstream>
#include <iostream>

using namespace game_engine;

game_engine::TrajectoryVector3D RunAstarPlusNumWaypointPolySolver(
    const Graph3D &graph, const std::shared_ptr<Node3D> start_ptr,
    const std::shared_ptr<Node3D> end_ptr, int derivative_order, int seg)