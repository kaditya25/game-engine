#include <chrono>
#include <queue>
#include <bits/stdc++.h>

#include "../dependencies/P4/dependencies/osqp/include/osqp.h"
#include "../dependencies/P4/src/polynomial_sampler.h"
#include "../dependencies/P4/src/polynomial_solver.h"
#include "../dependencies/P4/src/polynomial_bounds.h"
#include "../dependencies/P4/examples/gnuplot-iostream.h"

#include "multi_quad_autonomy_protocol.h"
#include "occupancy_grid3d.h"
#include "graph.h"
#include "student_game_engine_visualizer.h"

#define DISCRETE_LENGTH .4 // The length of one side of a cube in meters in the occupancy grid
#define SAFETY_BOUNDS .8  // How big the bubble around obstacles will be

namespace game_engine {

  namespace 
  {
    // Helper struct that functions as a linked list with data. The linked
    // list represents a path. Data members are a node, a cost to reach that
    // node, and a heuristic cost from the current node to the destination.
    struct NodeWrapper {
      std::shared_ptr<struct NodeWrapper> parent;
      std::shared_ptr<Node3D> node_ptr;

      // True cost to this node
      double cost;

      // Heuristic to end node
      double heuristic;

      // Equality operator
      bool operator==(const NodeWrapper& other) const {
        return *(this->node_ptr) == *(other.node_ptr);
      }
    };

    // Helper function. Compares the values of two NodeWrapper pointers.
    // Necessary for the priority queue.
    bool NodeWrapperPtrCompare(
        const std::shared_ptr<NodeWrapper>& lhs, 
        const std::shared_ptr<NodeWrapper>& rhs) {
      return lhs->cost + lhs->heuristic > rhs->cost + rhs->heuristic;
    }

    //Euclidean distance and straight bias heuristic
    double HeuristicStraightPathBias(
      const std::shared_ptr<Node3D>& par_ptr,
      const std::shared_ptr<Node3D>& cur_ptr,
      const std::shared_ptr<Node3D>& end_ptr) {
      
      double h;
      Eigen::Vector3d par(par_ptr->Data().x(),
        par_ptr->Data().y(),par_ptr->Data().z());
      Eigen::Vector3d cur(cur_ptr->Data().x(),
        cur_ptr->Data().y(),cur_ptr->Data().z());
      Eigen::Vector3d end(end_ptr->Data().x(),
        end_ptr->Data().y(),end_ptr->Data().z());

      Eigen::Vector3d v1 = cur-par;
      Eigen::Vector3d v2 = end-cur;

      double euc_dist = v2.norm();
      double pointing = v1.normalized().dot(v2.normalized());

      h = euc_dist - .1*pointing;

      return h;
    }

    std::vector<std::shared_ptr<Node3D>> Astar3D(std::shared_ptr<Node3D> start_ptr,
      std::shared_ptr<Node3D> end_ptr, Graph3D graphOfArena) {

      std:: cout << "RUNNING ASTAR3D" << std::endl;

      std::vector<std::shared_ptr<Node3D>> path;

      using NodeWrapperPtr = std::shared_ptr<NodeWrapper>;

      std::priority_queue<
        NodeWrapperPtr,
        std::vector<NodeWrapperPtr>,
        std::function<bool(
            const NodeWrapperPtr&, 
            const NodeWrapperPtr& )>> 
          to_explore(NodeWrapperPtrCompare);

      std::unordered_set<std::shared_ptr<Node3D>> explored;

      NodeWrapperPtr nw_ptr = std::make_shared<NodeWrapper>();
      nw_ptr->parent = nullptr;
      nw_ptr->node_ptr = start_ptr;
      nw_ptr->cost = 0;
      nw_ptr->heuristic = 0.0;
      to_explore.push(nw_ptr);

      NodeWrapperPtr node_to_explore;

      while(true){

        node_to_explore = to_explore.top();
        to_explore.pop();

        if (!(explored.find(node_to_explore->node_ptr) == explored.end())) {
          continue;
        }

        if(*(node_to_explore->node_ptr) == *end_ptr)
        {
          break;
        }
        else 
        {
          explored.insert(node_to_explore->node_ptr);
          const std::vector<DirectedEdge3D> edges = 
            graphOfArena.Edges(node_to_explore->node_ptr);

          for(const DirectedEdge3D& edge: edges)
          {
            const std::shared_ptr<Node3D>& sink_ptr = edge.Sink();
            const double cost = edge.Cost();

            NodeWrapperPtr neighbor = std::make_shared<NodeWrapper>();
            neighbor->parent = node_to_explore;
            neighbor->node_ptr = sink_ptr;
            neighbor->heuristic = HeuristicStraightPathBias(node_to_explore->node_ptr, neighbor->node_ptr, end_ptr);
            neighbor->cost = node_to_explore->cost + cost;
            to_explore.push(neighbor);
          }
        }

    }

      while(!(*(node_to_explore->node_ptr) == *start_ptr)){
        path.insert(path.begin(),node_to_explore->node_ptr);
        node_to_explore = node_to_explore->parent;
      }

      return path;

    }


    std::vector<std::shared_ptr<Node3D>> PrunePath(std::vector<std::shared_ptr<Node3D>> path)
    {
      std::vector<std::shared_ptr<Node3D>> pruned_path = path;
      int N = path.size();

      for(int i=N-2; i>0; i--)
      {
        Eigen::Vector3d p1(pruned_path[i-1]->Data().x(), 
          pruned_path[i-1]->Data().y(), pruned_path[i-1]->Data().z());
        Eigen::Vector3d p2(pruned_path[i]->Data().x(), 
          pruned_path[i]->Data().y(), pruned_path[i]->Data().z());
        Eigen::Vector3d p3(pruned_path[i+1]->Data().x(), 
          pruned_path[i+1]->Data().y(), pruned_path[i+1]->Data().z());

        Eigen::Vector3d v1 = p2 - p1;
        Eigen::Vector3d v2 = p3 - p2;
        if(v1.normalized().dot(v2.normalized())>.99)
        {
          pruned_path.erase(pruned_path.begin() + i);
        }
        else
        {
          continue;
        }

      }
      return pruned_path;
    }

    TrajectoryVector3D Trajectory2P4(Eigen::Vector3d current_position, Eigen::Vector3d current_velocity, 
      std::vector<Eigen::Vector3d> path_points, std::vector<double> times, int target)
    {
	    const std::chrono::time_point<std::chrono::system_clock> current_chrono_time
	    = std::chrono::system_clock::now();

	    std::vector<p4::NodeEqualityBound> node_equality_bounds = 
	    {
	    // The first node must constrain position, velocity, and acceleration
	    p4::NodeEqualityBound(0,0,0,current_position(0)),
	    p4::NodeEqualityBound(1,0,0,current_position(1)),
	    p4::NodeEqualityBound(2,0,0,current_position(2)),
	    p4::NodeEqualityBound(0,0,1,current_velocity(0)),
	    p4::NodeEqualityBound(1,0,1,current_velocity(1)),
	    p4::NodeEqualityBound(2,0,1,current_velocity(2)),
	    };

	    for(int i=0; i<path_points.size(); i++)
	    {
	    	node_equality_bounds.push_back(p4::NodeEqualityBound(0,i+1,0,path_points[i](0)));
	    	node_equality_bounds.push_back(p4::NodeEqualityBound(1,i+1,0,path_points[i](1)));
	    	node_equality_bounds.push_back(p4::NodeEqualityBound(2,i+1,0,path_points[i](2)));
	    }

	    std::vector<p4::NodeInequalityBound> node_Inequality_bounds;
	    if ((path_points.size() == 1) && ((target == 1) || (target ==2)))
	    {
	    	node_Inequality_bounds.push_back(p4::NodeInequalityBound(0,1,1,-0.5,0.5));
	    	node_Inequality_bounds.push_back(p4::NodeInequalityBound(1,1,1,-0.5,0.5));
	    	node_Inequality_bounds.push_back(p4::NodeInequalityBound(2,1,1,-0.5,0.5));
	    }
	    if ((path_points.size() == 1) && (target == 3))
	    {
	    	node_equality_bounds.push_back(p4::NodeEqualityBound(0,1,1,0));
	    	node_equality_bounds.push_back(p4::NodeEqualityBound(1,1,1,0));
	    	node_equality_bounds.push_back(p4::NodeEqualityBound(2,1,1,0));
	    }

	    p4::PolynomialSolver::Options solver_options;
	    solver_options.num_dimensions = 3;     // 3D
	    solver_options.polynomial_order = 8;   // Fit an 8th-order polynomial
	    solver_options.continuity_order = 4;   // Require continuity to the 4th order
	    solver_options.derivative_order = 2;   // Minimize acceleration

	    osqp_set_default_settings(&solver_options.osqp_settings);
	    solver_options.osqp_settings.polish = true;       // Polish the solution, getting the best answer possible
	    solver_options.osqp_settings.verbose = false;     // Suppress the printout

	    // Use p4::PolynomialSolver object to solve for polynomial trajectories
	    p4::PolynomialSolver solver(solver_options);
	    const p4::PolynomialSolver::Solution sol_path
	      = solver.Run(times, node_equality_bounds, {}, {});

	    p4::PolynomialSampler::Options sampler_options;
	    sampler_options.frequency = 100;             // Number of samples per second

	    sampler_options.derivative_order = 0;        // Derivative to sample (0 = pos)
	    p4::PolynomialSampler sampler_pos(sampler_options);
	    Eigen::MatrixXd pos_samples = sampler_pos.Run(times, sol_path);

	    sampler_options.derivative_order = 1;        // Update derivative order to velocity
	    p4::PolynomialSampler sampler_vel(sampler_options);
	    Eigen::MatrixXd vel_samples = sampler_vel.Run(times, sol_path);

	    sampler_options.derivative_order = 2;        // Update derivative order to acceleration
	    p4::PolynomialSampler sampler_acc(sampler_options);
	    Eigen::MatrixXd acc_samples = sampler_acc.Run(times, sol_path);

	    double yaw = 0;     // This is something we should change later

	    const int M = pos_samples.cols();
	    std::chrono::milliseconds dt_test = std::chrono::milliseconds(10);
	    double time;

	    TrajectoryVector3D trajectory_vector;
	    for(int idx = 0; idx < M; ++idx) 
	    {
	    	// chrono::duration<double> maintains high-precision floating point time
	    	// in seconds use the count function to cast into floating point
	    	std::chrono::duration<double> flight_chrono_time
	    	  = current_chrono_time.time_since_epoch() + idx * dt_test;
	    	// The trajectory requires the time to be specified as a floating point
	    	// number that measures the number of seconds since the unix epoch.

	    	time = flight_chrono_time.count();

	    	// Push an Eigen instance onto the trajectory vector
	    	trajectory_vector.push_back(
	    	    (Eigen::Matrix<double, 11, 1>() <<
	    	      pos_samples(1,idx), pos_samples(2,idx), pos_samples(3,idx),
	    	      vel_samples(1,idx), vel_samples(2,idx), vel_samples(3,idx),
	    	      acc_samples(1,idx), acc_samples(2,idx), acc_samples(3,idx),
	    	      yaw,
	    	      time
	    	      ).finished());
	    }
		return trajectory_vector;
    }

  }

  std::unordered_map<std::string, Trajectory>
  MultiQuadAutonomyProtocol::UpdateTrajectories() 
  {
  	const int M = friendly_names_.size();

	static OccupancyGrid3D occupancy_grid;
	static Graph3D graphOfArena;
	static std::vector<Student_game_engine_visualizer> visualizer(M);
	static std::vector<bool> initialize(M,true);
	static std::vector<bool> firstTime(M,true);

	static std::vector<Eigen::Vector3d> current_pos(M);
	static std::vector<Eigen::Vector3d> current_vel(M);
	static std::vector<Eigen::Vector3d> home_pos(M);
	static Eigen::Vector3d red_balloon_pos;
	static Eigen::Vector3d blue_balloon_pos;
	static std::shared_ptr<Node3D> home_ptr;
	static std::shared_ptr<Node3D> red_ptr;
	static std::shared_ptr<Node3D> blue_ptr;
	static std::vector<std::shared_ptr<Node3D>> current_ptr(M);
	static std::vector<int> target(M,0);
	static std::vector<std::shared_ptr<Node3D>> target_ptr(M);
	static std::vector<char> target_color(M);
	static std::vector<std::vector<std::shared_ptr<Node3D>>> path(M);

	static std::vector<bool> straight_to_target(M,false);
	static std::vector<int> next_path_node(M,0);
	
	static std::vector<double> seg_time(M,4);
	static std::vector<double> seg_adder(M,1);

	//if the quad comes within 1 meters of a node, consider it traveled to
	static const double threshold = 1.4;
	//Number of upcoming nodes that the quad uses to generate a trajectory
	static const int nodes_in_traj = 5;
	//Total number of nodes that will be planned by A star
	static const int nodes_to_search = 10;
	//The base segment time in seconds
	static const double def_seg_time = 6.5;
	//Amount of time added to def_seg_time if the trajectory is
	//rejected for vel/accel reasons
	static const double def_seg_adder = 1.5;

	//Segment time when the quad goes straight for the target
	static const double seg_time_straight_to_target = 2;
	//Amount of time added to seg_time_straight_to_target
	//if trajectory is rejected for vel/accel reasons
	static const double seg_adder_straight_to_target = 1;

	TrajectoryVector3D trajectory_vector;

	std::unordered_map<std::string, Trajectory> trajectory_map;

  	for(size_t ii=0; ii<friendly_names_.size(); ii++)
  	{	
	    std::string& quad_name = friendly_names_[ii];
	    this->snapshot_->Position(quad_name, current_pos[ii]);
	    if (initialize[ii])
	    {
	    	initialize[ii] = false;
	    	trajectoryCodeMap_[quad_name].code = MediationLayerCode::NotEnoughTrajectoryPoints;

	    	//create the grid/graph and visualize
	    	occupancy_grid.LoadFromMap(map3d_, DISCRETE_LENGTH, SAFETY_BOUNDS);
	    	graphOfArena = occupancy_grid.AsGraph();  // You can run Astar on this graph
	    	visualizer[ii].startVisualizing("/game_engine/environment");

	    	//get the home position of the quad and positions of the balloons
	    	home_pos[ii] = current_pos[ii];
	    	red_balloon_pos = *red_balloon_position_;
	    	blue_balloon_pos = *blue_balloon_position_;
	    	//convert them to grid coords for Astar
	    	std::tuple<int,int,int> home_grid = occupancy_grid.mapToGridCoordinates(home_pos[ii]);
	    	home_ptr = std::make_shared<Node3D>(Eigen::Vector3d(
				std::get<0>(home_grid),std::get<1>(home_grid),std::get<2>(home_grid)));

	    	std::tuple<int,int,int> red_grid = occupancy_grid.mapToGridCoordinates(red_balloon_pos);
	    	red_ptr = std::make_shared<Node3D>(Eigen::Vector3d(
				std::get<0>(red_grid),std::get<1>(red_grid),std::get<2>(red_grid)));

	    	std::tuple<int,int,int> blue_grid = occupancy_grid.mapToGridCoordinates(blue_balloon_pos);
	    	blue_ptr = std::make_shared<Node3D>(Eigen::Vector3d(
				std::get<0>(blue_grid),std::get<1>(blue_grid),std::get<2>(blue_grid)));
	    }	
	    
	    //Make it so apollo is heading for target 1 (red balloon) and
	    //zeus is heading for target 2 (blue balloon)
	    if(quad_name == "apollo")
	    {
	    	target_ptr[ii] = red_ptr;
	    	target_color[ii] = 'r';
	    	target[ii] = 1;
	    }
	    if(quad_name == "zeus")
	    {
	    	target_ptr[ii] = blue_ptr;
	    	target_color[ii] = 'b';
	    	target[ii] = 2;
	    }

	    if (firstTime[ii]) 
	    {
			firstTime[ii] = false;
			std::tuple<int,int,int> cur_grid = occupancy_grid.mapToGridCoordinates(current_pos[ii]);
			current_ptr[ii] = std::make_shared<Node3D>(Eigen::Vector3d(
				std::get<0>(cur_grid),std::get<1>(cur_grid),std::get<2>(cur_grid)));

			path[ii] = Astar3D(current_ptr[ii], target_ptr[ii], graphOfArena);
			std::cout << "ASTAR COMPLETE" << std::endl;
			path[ii] = PrunePath(path[ii]);
			next_path_node[ii] = 0;
			seg_time[ii] = def_seg_time;
			seg_adder[ii] = def_seg_adder;
			trajectoryCodeMap_[quad_name].code = MediationLayerCode::NotEnoughTrajectoryPoints;
	    }

	    int N = path[ii].size();
	    int target_node = N-1;
	    //current quad position
	    this->snapshot_->Velocity(quad_name, current_vel[ii]);
	    //start looking at the next waypoint
	    int i0 = next_path_node[ii];
	    
	    for(int i = i0; i<(i0+nodes_to_search); i++)
	    {
			if (i>=N-1) break;
			// Check if point has been hit
			Eigen::Vector3d path_point = occupancy_grid.boxCenter(path[ii][i]->Data().x(),
			path[ii][i]->Data().y(), path[ii][i]->Data().z());
			Eigen::Vector3d diff = path_point - current_pos[ii];
			if(diff.norm() < threshold)
			{
				next_path_node[ii] = i+1;
				seg_time[ii] = def_seg_time;
				seg_adder[ii] = def_seg_adder;

				std::cout << "         NEXT UP: " << next_path_node[ii] << std::endl;

				if (next_path_node[ii] == target_node) 
				{
					straight_to_target[ii] = true;
					seg_time[ii] = seg_time_straight_to_target;
					seg_adder[ii] = seg_adder_straight_to_target;
				} 
				else 
				{
					Eigen::Vector3d path_point_np1 = occupancy_grid.boxCenter(path[ii][i+2]->Data().x(),
					  path[ii][i+2]->Data().y(), path[ii][i+2]->Data().z());
					double time_dist = (path_point_np1 - current_pos[ii]).norm();
					std::cout << "Distance from current pos to end node: " << time_dist << std::endl;
				}
			}
	    }// end for loop  

	    //guess the segment time as very low. if the last one was a success, start low again
	    if(trajectoryCodeMap_[quad_name].code != MediationLayerCode::Success)
	    {
			std::cout << "Response code: " <<
			static_cast<unsigned int>(trajectoryCodeMap_[quad_name].code) << std::endl;
	    }
	    if(trajectoryCodeMap_[quad_name].code == MediationLayerCode::Success)
	    {
			std::cout << "SUCCESSFUL TRJECTORY SUBMISSION" << std::endl;
			seg_time[ii] = 1;
			seg_adder[ii]  = 0;
	    }
	    if((trajectoryCodeMap_[quad_name].code == MediationLayerCode::ExceedsMaxVelocity) ||
			(trajectoryCodeMap_[quad_name].code == MediationLayerCode::MeanValueExceedsMaxVelocity) ||
			(trajectoryCodeMap_[quad_name].code == MediationLayerCode::ExceedsMaxAcceleration) ||
			(trajectoryCodeMap_[quad_name].code == MediationLayerCode::MeanValueExceedsMaxAcceleration)) 
	    {
			seg_time[ii] = seg_time[ii] + seg_adder[ii];
			std::cout << "TOO FAST. NEW SEG TIME" << std::endl;
	    }

	    int segs_until_end;
	    if (next_path_node[ii] == N-1)
	    {
			segs_until_end = 1;
	    } 
	    else if (next_path_node[ii] == N-2)
		{
			segs_until_end = 2;
	    } 
	    else
	    {
			segs_until_end = 3;
	    }

	    double time_between_nodes = seg_time[ii]/segs_until_end;
	    std::vector<double> times = {0};
	    std::vector<Eigen::Vector3d> path_points;

	    if (!straight_to_target[ii])
	    {
			for(int i=0; i<nodes_in_traj; i++)
			{
				if (next_path_node[ii]+i >= N) break;
				times.push_back((i+1)*time_between_nodes);
				path_points.push_back(occupancy_grid.boxCenter(path[ii][next_path_node[ii]+i]->Data().x(),
		  		  path[ii][next_path_node[ii]+i]->Data().y(), path[ii][next_path_node[ii]+i]->Data().z()));
			}
			trajectory_vector = Trajectory2P4(current_pos[ii], current_vel[ii], path_points, times, target[ii]);
	    }
	    else 
	    {
			//go straight at target
			std::cout << "SENDING IT TO TARGET" << std::endl;
			times.push_back(time_between_nodes);
			if (target_color[ii] == 'b')
			{
				path_points.push_back(blue_balloon_pos);
			} 
			else
			{
				path_points.push_back(red_balloon_pos);
			}
			trajectory_vector = Trajectory2P4(current_pos[ii], current_vel[ii], path_points, times, target[ii]);
	    }

	    Trajectory trajectory(trajectory_vector);
	    visualizer[ii].drawTrajectory(trajectory);
	    trajectory_map[quad_name] = trajectory;
    }//End for loop
    return trajectory_map;
  }//End UpdateTrajectories() function

}
