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

void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<double>>> dataset);

namespace game_engine {

    namespace {
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

        //Euclidean distance heuristic
        double Heuristic_euclid(
                const std::shared_ptr<Node3D>& current_ptr,
                const std::shared_ptr<Node3D>& end_ptr) {

            float h;
            float x_cur = current_ptr->Data().x();
            float y_cur = current_ptr->Data().y();
            float z_cur = current_ptr->Data().z();
            float x_end = end_ptr->Data().x();
            float y_end = end_ptr->Data().y();
            float z_end = end_ptr->Data().z();

            h = sqrt(pow(x_end-x_cur,2)+pow(y_end-y_cur,2)+pow(z_end-z_cur,2));

            return h;
        }

        //Euclidean distance and straight bias heuristic
        double Heuristic_new(
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

                if(*(node_to_explore->node_ptr) == *end_ptr){
                    break;
                }else {
                    explored.insert(node_to_explore->node_ptr);
                    const std::vector<DirectedEdge3D> edges =
                            graphOfArena.Edges(node_to_explore->node_ptr);

                    for(const DirectedEdge3D& edge: edges){
                        const std::shared_ptr<Node3D>& sink_ptr = edge.Sink();
                        const double cost = edge.Cost();

                        NodeWrapperPtr neighbor = std::make_shared<NodeWrapper>();
                        neighbor->parent = node_to_explore;
                        neighbor->node_ptr = sink_ptr;
                        neighbor->heuristic = Heuristic_new(node_to_explore->node_ptr, neighbor->node_ptr, end_ptr);
                        // neighbor->heuristic = Heuristic_euclid(neighbor->node_ptr,end_ptr);
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


        std::vector<std::shared_ptr<Node3D>> prune(std::vector<std::shared_ptr<Node3D>> path)
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

        TrajectoryVector3D peeFour(Eigen::Vector3d current_pos, Eigen::Vector3d current_vel,
                                   std::vector<Eigen::Vector3d> path_points, std::vector<double> times, int target){

            const std::chrono::time_point<std::chrono::system_clock> current_chrono_time
                    = std::chrono::system_clock::now();

            std::vector<p4::NodeEqualityBound> node_equality_bounds = {
                    // The first node must constrain position, velocity, and acceleration
                    p4::NodeEqualityBound(0,0,0,current_pos(0)),
                    p4::NodeEqualityBound(1,0,0,current_pos(1)),
                    p4::NodeEqualityBound(2,0,0,current_pos(2)),
                    p4::NodeEqualityBound(0,0,1,current_vel(0)),
                    p4::NodeEqualityBound(1,0,1,current_vel(1)),
                    p4::NodeEqualityBound(2,0,1,current_vel(2)),
            };

            for(int i=0; i<path_points.size(); i++)
            {
                node_equality_bounds.push_back(p4::NodeEqualityBound(0,i+1,0,path_points[i](0)));
                node_equality_bounds.push_back(p4::NodeEqualityBound(1,i+1,0,path_points[i](1)));
                node_equality_bounds.push_back(p4::NodeEqualityBound(2,i+1,0,path_points[i](2)));
            }

            std::vector<p4::NodeInequalityBound> node_Inequality_bounds;
            if ((path_points.size() == 1) && ((target == 1) || (target ==2))) {
                node_Inequality_bounds.push_back(p4::NodeInequalityBound(0,1,1,-0.5,0.5));
                node_Inequality_bounds.push_back(p4::NodeInequalityBound(1,1,1,-0.5,0.5));
                node_Inequality_bounds.push_back(p4::NodeInequalityBound(2,1,1,-0.5,0.5));
            }
            if ((path_points.size() == 1) && (target == 3)){
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
            Eigen::MatrixXd Pos_samples = sampler_pos.Run(times, sol_path);

            sampler_options.derivative_order = 1;        // Update derivative order to velocity
            p4::PolynomialSampler sampler_vel(sampler_options);
            Eigen::MatrixXd Vel_samples = sampler_vel.Run(times, sol_path);

            sampler_options.derivative_order = 2;        // Update derivative order to acceleration
            p4::PolynomialSampler sampler_acc(sampler_options);
            Eigen::MatrixXd Acc_samples = sampler_acc.Run(times, sol_path);

            double yaw = 0;     // This is something we should change later

            const int M = Pos_samples.cols();
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
                                                        Pos_samples(1,idx), Pos_samples(2,idx), Pos_samples(3,idx),
                                Vel_samples(1,idx), Vel_samples(2,idx), Vel_samples(3,idx),
                                Acc_samples(1,idx), Acc_samples(2,idx), Acc_samples(3,idx),
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

        std::unordered_map<std::string, Trajectory> trajectory_map;

        for(size_t ii=0; ii<friendly_names_.size(); ii++)
        {

            static OccupancyGrid3D occupancy_grid;
            static Graph3D graphOfArena;   // Used by A*, need to convert A* 2d to 3D
            static Student_game_engine_visualizer visualizer;
            static bool firstTime_one = true;
            static bool firstTime_two = true;
            static bool firstTime_three = true;
            static bool initialize = true;
            std::string& quad_name = friendly_names_[ii];
            std::cout << quad_name << std::endl;
            static Eigen::Vector3d current_pos;
            static Eigen::Vector3d current_vel;
            static Eigen::Vector3d home_pos;
            static Eigen::Vector3d red_balloon_pos;
            static Eigen::Vector3d blue_balloon_pos;
            static std::shared_ptr<Node3D> home_ptr;
            static std::shared_ptr<Node3D> red_ptr;
            static std::shared_ptr<Node3D> blue_ptr;
            static std::shared_ptr<Node3D> current_ptr;
            static std::shared_ptr<Node3D> target1_ptr;
            static std::shared_ptr<Node3D> target2_ptr;
            static char target1_color;
            static char target2_color;
            static std::vector<std::shared_ptr<Node3D>> path_one;
            static std::vector<std::shared_ptr<Node3D>> path_two;
            static std::vector<std::shared_ptr<Node3D>> path_three;
            static std::vector<std::shared_ptr<Node3D>> path_tot;
            //if the quad comes within 1 meters of a node, consider it traveled to
            static const double threshold = 1.4;
            //look ahead 3 nodes
            static const int nodes_in_traj = 2;
            static const int nodes_to_search = 3;
            //start at the first target
            static int target = 1;
            static bool send_it = false;
            static int next_up = 0;
            static double seg_time = 4;
            static double seg_adder = 1;
            static const double def_seg_time = 6.5;
            static const double def_seg_adder = 1.5;
            static const double seg_time_send = 2;
            static const double seg_adder_send = 1;
            TrajectoryVector3D trajectory_vector;

            this->snapshot_->Position(quad_name, current_pos);

            if (initialize){
                initialize = false;
                trajectoryCode_.code = MediationLayerCode::NotEnoughTrajectoryPoints;

                //create the grid/graph and visualize
                occupancy_grid.LoadFromMap(map3d_, DISCRETE_LENGTH, SAFETY_BOUNDS);
                graphOfArena = occupancy_grid.AsGraph();  // You can run Astar on this graph
                visualizer.startVisualizing("/game_engine/environment");

                //get the home position of the quad and positions of the balloons
                home_pos = current_pos;
                red_balloon_pos = red_balloon_status_->position;
                blue_balloon_pos = blue_balloon_status_->position;

                //convert them to grid coords for Astar
                std::tuple<int,int,int> home_grid = occupancy_grid.mapToGridCoordinates(home_pos);
                home_ptr = std::make_shared<Node3D>(Eigen::Vector3d(
                        std::get<0>(home_grid),std::get<1>(home_grid),std::get<2>(home_grid)));
                std::tuple<int,int,int> red_grid = occupancy_grid.mapToGridCoordinates(red_balloon_pos);
                red_ptr = std::make_shared<Node3D>(Eigen::Vector3d(
                        std::get<0>(red_grid),std::get<1>(red_grid),std::get<2>(red_grid)));
                std::tuple<int,int,int> blue_grid = occupancy_grid.mapToGridCoordinates(blue_balloon_pos);
                blue_ptr = std::make_shared<Node3D>(Eigen::Vector3d(
                        std::get<0>(blue_grid),std::get<1>(blue_grid),std::get<2>(blue_grid)));

                target1_ptr = red_ptr;
                target1_color = 'r';
                target2_ptr = blue_ptr;
                target2_color = 'b';
            }

            //Make it so apollo is heading for target 1 (red balloon) and
            //zeus is heading for target 2 (blue balloon)
            if(quad_name == "apollo")
            {
                target = 1;
            }
            if(quad_name == "zeus")
            {
                target = 2;
            }

            if (target == 1) {

                if (firstTime_one) {
                    firstTime_one = false;
                    std::tuple<int,int,int> cur_grid = occupancy_grid.mapToGridCoordinates(current_pos);
                    current_ptr = std::make_shared<Node3D>(Eigen::Vector3d(
                            std::get<0>(cur_grid),std::get<1>(cur_grid),std::get<2>(cur_grid)));
                    path_one = Astar3D(current_ptr, target1_ptr, graphOfArena);
                    std::cout << "ASTAR COMPLETE" << std::endl;
                    path_one = prune(path_one);
                    send_it = false;
                    next_up = 0;
                    seg_time = def_seg_time;
                    seg_adder = def_seg_adder;
                    trajectoryCode_.code = MediationLayerCode::NotEnoughTrajectoryPoints;
                }

                const int N = path_one.size();
                std::cout << "nodes in traj: " << N << std::endl;
                int target_node = N-1;
                //current quad position
                this->snapshot_->Velocity(quad_name, current_vel);
                //start looking at the next waypoint
                int i0 = next_up;
                for(int i = i0; i<(i0+nodes_to_search); i++){
                    if (i>=N-1) break;
                    // Check if point has been hit
                    Eigen::Vector3d path_point = occupancy_grid.boxCenter(path_one[i]->Data().x(),
                                                                          path_one[i]->Data().y(), path_one[i]->Data().z());
                    Eigen::Vector3d diff = path_point - current_pos;
                    if(diff.norm() < threshold){
                        next_up = i+1;
                        seg_time = def_seg_time;
                        seg_adder = def_seg_adder;
                        std::cout << "-----------------------------------------"<< std::endl;
                        std::cout << "         NEXT UP: " << next_up << std::endl;
                        std::cout << "-----------------------------------------"<< std::endl;
                        if (next_up == target_node) {
                            send_it = true;
                            seg_time = 1;
                            seg_time = seg_time_send;
                            seg_adder = seg_adder_send;
                        } else {
                            Eigen::Vector3d path_point_np1 = occupancy_grid.boxCenter(path_one[i+2]->Data().x(),
                                                                                      path_one[i+2]->Data().y(), path_one[i+2]->Data().z());
                            double time_dist = (path_point_np1 - current_pos).norm();
                            std::cout << "Distance from current pos to end node: " << time_dist << std::endl;
                        }
                    }
                }// end for loop

                //guess the sesgment time as very low. if the last one was a success, start low again
                if (trajectoryCode_.code != MediationLayerCode::Success) {
                    std::cout << "Response code: " <<
                              static_cast<unsigned int>(trajectoryCode_.code) << std::endl;
                }
                if (trajectoryCode_.code == MediationLayerCode::Success){
                    std::cout << "SUCCESSFUL TRJECTORY SUBMISSION" << std::endl;
                    seg_time = 1;
                    seg_adder  = 0;
                }
                if ((trajectoryCode_.code == MediationLayerCode::ExceedsMaxVelocity) ||
                    (trajectoryCode_.code == MediationLayerCode::MeanValueExceedsMaxVelocity) ||
                    (trajectoryCode_.code == MediationLayerCode::ExceedsMaxAcceleration) ||
                    (trajectoryCode_.code == MediationLayerCode::MeanValueExceedsMaxAcceleration)) {
                    seg_time = seg_time + seg_adder;
                    std::cout << "TOO FAST. NEW SEG TIME" << std::endl;
                }

                std::cout << "seg_time: " << seg_time << std::endl;
                std::cout << "seg_adder: "<< seg_adder << std::endl;

                int segs;
                if (next_up == N-1) {
                    segs = 1;
                } else if (next_up == N-2) {
                    segs = 2;
                } else {
                    segs = 3;
                }

                double time_between_nodes = seg_time/segs;
                std::vector<double> times = {0};
                std::vector<Eigen::Vector3d> path_points;

                if (!send_it){

                    for(int i=0; i<nodes_in_traj; i++)
                    {
                        if (next_up+i >= N) break;
                        times.push_back((i+1)*time_between_nodes);
                        path_points.push_back(occupancy_grid.boxCenter(path_one[next_up+i]->Data().x(),
                                                                       path_one[next_up+i]->Data().y(), path_one[next_up+i]->Data().z()));
                    }

                    trajectory_vector = peeFour(current_pos, current_vel, path_points, times, target);

                }else {
                    //go straight at target
                    std::cout << "SENDING IT TO TARGET" << std::endl;

                    times.push_back(time_between_nodes);
                    if (target1_color == 'b') {
                        path_points.push_back(blue_balloon_pos);
                    } else {
                        path_points.push_back(red_balloon_pos);
                    }

                    trajectory_vector = peeFour(current_pos, current_vel, path_points, times, target);

                }

            }

            //SECOND TARGET
            if (target == 2) {

                if (firstTime_two)
                {
                    firstTime_two = false;
                    std::tuple<int,int,int> cur_grid = occupancy_grid.mapToGridCoordinates(current_pos);
                    current_ptr = std::make_shared<Node3D>(Eigen::Vector3d(
                            std::get<0>(cur_grid),std::get<1>(cur_grid),std::get<2>(cur_grid)));
                    path_two = Astar3D(current_ptr, target2_ptr, graphOfArena);
                    std::cout << "ASTAR COMPLETE" << std::endl;
                    path_two = prune(path_two);
                    send_it = false;
                    next_up = 0;
                    seg_time = def_seg_time;
                    seg_adder = def_seg_adder;
                    trajectoryCode_.code = MediationLayerCode::NotEnoughTrajectoryPoints;
                }

                const int N = path_two.size();
                std::cout << "nodes in traj: " << N << std::endl;
                int target_node = N-1;
                //current quad position
                this->snapshot_->Velocity(quad_name, current_vel);
                //start looking at the next waypoint
                int i0 = next_up;
                for(int i = i0; i<(i0+nodes_to_search); i++){
                    if (i>=N-1) break;
                    // Check if point has been hit
                    Eigen::Vector3d path_point = occupancy_grid.boxCenter(path_two[i]->Data().x(),
                                                                          path_two[i]->Data().y(), path_two[i]->Data().z());
                    Eigen::Vector3d diff = path_point - current_pos;
                    if(diff.norm() < threshold){
                        next_up = i+1;
                        seg_time = def_seg_time;
                        seg_adder = def_seg_adder;
                        std::cout << "-----------------------------------------"<< std::endl;
                        std::cout << "         NEXT UP: " << next_up << std::endl;
                        std::cout << "-----------------------------------------"<< std::endl;
                        if (next_up == target_node) {
                            send_it = true;
                            seg_time = seg_time_send;
                            seg_adder = seg_adder_send;
                        } else {
                            Eigen::Vector3d path_point_np1 = occupancy_grid.boxCenter(path_two[i+2]->Data().x(),
                                                                                      path_two[i+2]->Data().y(), path_two[i+2]->Data().z());
                            double time_dist = (path_point_np1 - current_pos).norm();
                            std::cout << "Distance from current pos to end node: " << time_dist << std::endl;
                        }
                    }
                }// end for loop

                //guess the sesgment time as very low. if the last one was a success, start low again
                if (trajectoryCode_.code != MediationLayerCode::Success) {
                    std::cout << "Response code: " <<
                              static_cast<unsigned int>(trajectoryCode_.code) << std::endl;
                }
                if (trajectoryCode_.code == MediationLayerCode::Success){
                    std::cout << "SUCCESSFUL TRJECTORY SUBMISSION" << std::endl;
                    seg_time = 1;
                    seg_adder  = 0;
                }
                if ((trajectoryCode_.code == MediationLayerCode::ExceedsMaxVelocity) ||
                    (trajectoryCode_.code == MediationLayerCode::MeanValueExceedsMaxVelocity) ||
                    (trajectoryCode_.code == MediationLayerCode::ExceedsMaxAcceleration) ||
                    (trajectoryCode_.code == MediationLayerCode::MeanValueExceedsMaxAcceleration)) {
                    seg_time = seg_time + seg_adder;
                    std::cout << "TOO FAST. NEW SEG TIME" << std::endl;
                }

                std::cout << "seg_time: " << seg_time << std::endl;
                std::cout << "seg_adder: "<< seg_adder << std::endl;

                int segs;
                if (next_up == N-1) {
                    segs = 1;
                } else if (next_up == N-2) {
                    segs = 2;
                } else {
                    segs = 3;
                }

                double time_between_nodes = seg_time/segs;
                std::vector<double> times = {0};
                std::vector<Eigen::Vector3d> path_points;

                if (!send_it){

                    for(int i=0; i<nodes_in_traj; i++)
                    {
                        if (next_up+i >= N) break;
                        times.push_back((i+1)*time_between_nodes);
                        path_points.push_back(occupancy_grid.boxCenter(path_two[next_up+i]->Data().x(),
                                                                       path_two[next_up+i]->Data().y(), path_two[next_up+i]->Data().z()));
                    }

                    trajectory_vector = peeFour(current_pos, current_vel, path_points, times, target);

                }else {
                    //go straight at target
                    std::cout << "SENDING IT TO TARGET" << std::endl;

                    times.push_back(time_between_nodes);
                    if (target2_color == 'b') {
                        path_points.push_back(blue_balloon_pos);
                    } else {
                        path_points.push_back(red_balloon_pos);
                    }

                    trajectory_vector = peeFour(current_pos, current_vel, path_points, times, target);

                }

            }

            Trajectory trajectory(trajectory_vector);
            visualizer.drawTrajectory(trajectory);
            trajectory_map[quad_name] = trajectory;
        }//End for loop
        return trajectory_map;
    }

}
