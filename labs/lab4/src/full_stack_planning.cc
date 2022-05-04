#include <cstdlib>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include "a_star2d.h"
#include "occupancy_grid2d.h"
#include "path_info.h"
#include "polynomial_solver.h"
#include "polynomial_sampler.h"
#include "gnuplot-iostream.h"
#include "gui2d.h"

using namespace game_engine;

// Create a CSV file for MATLAB
void convert2CSV(std::string file_name, Eigen::MatrixXd matrix)
{
  std::cout << "The file: " << file_name.c_str() << std::endl;
  std::ofstream file(file_name.c_str());
  if (file.is_open())
  {
    file << matrix << '\n';
  }
}

// Polynomial planner parameter configuration
Eigen::MatrixXd sample(const std::vector<double> &times, const p4::PolynomialSolver::Solution &path, double frq, size_t option, std::string file)
{
  p4::PolynomialSampler::Options sampler_options;
  sampler_options.frequency = frq;             
  sampler_options.derivative_order = option;    

  // Use this object to sample a trajectory
  p4::PolynomialSampler sampler(sampler_options);
  Eigen::MatrixXd samples = sampler.Run(times, path);

  std::cout << "Writting to file..." << std::endl;
  convert2CSV( file, samples);
  std::cout << "Done." << std::endl;

  return samples;
}

int main(int argc, char** argv) {
  if(argc != 6) {
    std::cerr << "Usage: ./full_stack_planning occupancy_grid_file row1 col1 row2 col2" << std::endl;
    return EXIT_FAILURE;
  }

  // Parse the CLI input
  const std::string occupancy_grid_file = argv[1];
  const std::shared_ptr<Node2D> start_ptr = std::make_shared<Node2D>(Eigen::Vector2d(std::stoi(argv[2]),std::stoi(argv[3])));
  const std::shared_ptr<Node2D> goal_ptr = std::make_shared<Node2D>(Eigen::Vector2d(std::stoi(argv[4]),std::stoi(argv[5])));

  // Load an occupancy grid from a file
  OccupancyGrid2D occupancy_grid;
  occupancy_grid.LoadFromFile(occupancy_grid_file);

  // Transform an occupancy grid into a graph
  const Graph2D graph = occupancy_grid.AsGraph();

  /////////////////////////////////////////////////////////////////////////////
  // RUN A STAR
  //       Run your A* implementation over the graph and nodes defined above.
  //       This section is intended to be more free-form. Using previous
  //       problems and examples, determine the correct commands to complete
  //       this problem. You may want to take advantage of some of the plotting
  //       and graphing utilities in previous problems to check your solution on
  //       the way.
  /////////////////////////////////////////////////////////////////////////////
  AStar2D a_star;
  PathInfo path_info = a_star.Run(graph, start_ptr, goal_ptr);

  /////////////////////////////////////////////////////////////////////////////
  // RUN THE POLYNOMIAL PLANNER
  //       Convert the A* solution to a problem the polynomial solver can
  //       solve. Solve the polynomial problem, sample the solution, figure out
  //       a way to export it to Matlab.
  /////////////////////////////////////////////////////////////////////////////

  
  std::vector<double> times;

  // (dimension_index, node_idx, derivative_idx, value)
  std::vector<p4::NodeEqualityBound> node_equality_bounds = {
  // The first node must constrain position, velocity, and acceleration
      p4::NodeEqualityBound(0,0,1,0),
      p4::NodeEqualityBound(1,0,1,0),
      p4::NodeEqualityBound(0,0,2,0),
      p4::NodeEqualityBound(1,0,2,0),  
  };

  int i = 0;
  const double step = 1;
  for(const std::shared_ptr<Node2D> node: path_info.path)
  {
    times.push_back(i*step);
    node_equality_bounds.push_back( 
      p4::NodeEqualityBound( 0, i, 0, node->Data().x() )
    );
    node_equality_bounds.push_back( 
      p4::NodeEqualityBound( 1, i, 0, node->Data().y() )
    );
    i++;
  }

  p4::PolynomialSolver::Options solver_options;
  solver_options.num_dimensions = 2;     // 2D
  solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
  solver_options.continuity_order = 4;   // Require continuity to the 4th order
  solver_options.derivative_order = 2;   // Minimize acceleration

  osqp_set_default_settings(&solver_options.osqp_settings);
  solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
  solver_options.osqp_settings.verbose = false;     // Suppress the printout

  p4::PolynomialSolver solver(solver_options);
  const p4::PolynomialSolver::Solution path
    = solver.Run(times, node_equality_bounds, {}, {});

    const double sampling_rate = 200;
    Eigen::MatrixXd samples = sample(times, path, sampling_rate, 0, "/home/aeronaut/Workspace/game-engine/labs/lab4/pos.csv");
    sample(times, path, sampling_rate, 1, "/home/aeronaut/Workspace/game-engine/labs/lab4/vel.csv");
    sample(times, path, sampling_rate, 2, "/home/aeronaut/Workspace/game-engine/labs/lab4/accel.csv");
    
  return EXIT_SUCCESS;
}
