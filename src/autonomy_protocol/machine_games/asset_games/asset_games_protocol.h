#pragma once

#include "autonomy_protocol.h"

#include <chrono>
#include <queue>
#include <bits/stdc++.h>
#include "occupancy_grid3d.h"
#include "graph.h"
#include "student_game_engine_visualizer.h"
#include "realtime_game_theoretic_planner/helper.h"
#include "realtime_game_theoretic_planner/controller.h"
#include "realtime_game_theoretic_planner/nash_controller.h"
#include "realtime_game_theoretic_planner/vel_match_controller.h"

enum ControlCode 
{
  NashGame =0,
  VelMatch =1,
};

using costFunction = std::function<void(double&,
    const Eigen::VectorXd&, // x_self
    const Eigen::VectorXd&, // u_self
    const Eigen::VectorXd&, // x_other
    const Eigen::VectorXd&  // u_other
    )>;

using dynFunction = std::function<void(Eigen::VectorXd&, // x_k+1
    const Eigen::VectorXd&, // x_k
    const Eigen::VectorXd&, // u
    const double&           // dt
    )>;

namespace game_engine {
  class AssetGamesProtocol : public AutonomyProtocol {

    public:
      AssetGamesProtocol(
          const std::vector<std::string>& friendly_names,
          const std::vector<std::string>& enemy_names,
          const std::shared_ptr<GameSnapshot> snapshot,
          const std::shared_ptr<TrajectoryWardenClient> trajectory_warden_client,
          const std::shared_ptr<PreSubmissionTrajectoryVetter> prevetter,
          const Map3D& map3d,
          const std::shared_ptr<BalloonStatus> red_balloon_status,
          const std::shared_ptr<BalloonStatus> blue_balloon_status,
          const Eigen::Vector3d& goal_position,
          const WindIntensity& wind_intensity,
          std::vector<ControlCode> control_codes      )
        : AutonomyProtocol(
            friendly_names,
            enemy_names,
            snapshot,
            trajectory_warden_client,
            prevetter,
            map3d.Inflate(1),
            red_balloon_status,
            blue_balloon_status,
            goal_position,
            wind_intensity)   ,

        pursuer_state_(4),
        evader_state_(4),
        pursuer_total_state_(6),
        evader_total_state_(6),
        target_(2),
        u_pursuer_enum_( constAccCtrlInput(acc_max_pursuer_,num_pursuer_enum_)),
        u_evader_enum_ ( constAccCtrlInput(acc_max_evader_,num_evader_enum_))
        { 
          if (control_codes.size()!=2)
          { std::cout << "incorrect number of control codes!\n"; }
          // create and pass ownership of controllers
          switch(control_codes[0]) 
          {
            case NashGame:
              {
                std::unique_ptr<Controller> ctrl(new nashController(
                      u_pursuer_enum_,
                      u_evader_enum_,
                      K_start_, K_steps_, dt_,
                      pursuer_state_, evader_state_,
                      pursuer_dyn_functor_,
                      evader_dyn_functor_,
                      pursuer_cost_functor_,
                      evader_cost_functor_,
                      vel_max_pursuer_, acc_max_pursuer_, target_
                      ));
                controllers_.push_back(std::move(ctrl));
                break;
              }
            case VelMatch:
              {
                std::unique_ptr<Controller> ctrl(new velMatchController(
                      pursuer_state_,
                      evader_state_,
                      target_,
                      acc_max_pursuer_,
                      vel_max_pursuer_, 
                      dt_,
                      Role::pursuer
                      ));
                controllers_.push_back(std::move(ctrl));
                break;
              }
          }

          switch(control_codes[1]) 
          {
            case NashGame:
              {
                std::unique_ptr<Controller> ctrl(new nashController(
                      u_evader_enum_,
                      u_pursuer_enum_,
                      K_start_, K_steps_, dt_,
                      evader_state_,
                      pursuer_state_, 
                      evader_dyn_functor_,
                      pursuer_dyn_functor_,
                      evader_cost_functor_,
                      pursuer_cost_functor_,
                      vel_max_evader_, acc_max_evader_, target_
                      ));
                controllers_.push_back(std::move(ctrl));
                break;
              }
            case VelMatch:
              {
                std::unique_ptr<Controller> ctrl(new velMatchController(
                      evader_state_,
                      pursuer_state_,
                      target_,
                      acc_max_evader_,
                      vel_max_evader_, 
                      dt_,
                      Role::evader
                      ));
                controllers_.push_back(std::move(ctrl));
                break;
              }
          }

          vis_.startVisualizing("/game_engine/environment");
        }

      std::unordered_map<std::string, Trajectory> UpdateTrajectories() override;

    private:

      // for obstacle avoidance via potential-field-like
      // cost impositions
      const double obstacle_crit_dist_ = 2;
      const double obstacle_gain_      = 1;

      // acceleration and velocity constraints
      const double acc_max_pursuer_  = 0.3;
      const double acc_max_evader_  = 0.3;
      const double vel_max_pursuer_  = 1.25;
      const double vel_max_evader_  = 1.25;
      // discrete time system parameters
      const double dt_       = 1.25;
      const int K_steps_      = 2;
      const int num_pursuer_enum_   = 8;
      const int num_evader_enum_    = 8;
      // not actually used, but could be potentially useful
      int K_start_            = 1;

      // unique_ptr used, since the pointer needs to be created in the constructor
      // but the object itself will go out of scope without passing ownership
      std::vector<std::unique_ptr<Controller>> controllers_;

      // controller states and control inputs
      Eigen::VectorXd pursuer_state_;
      Eigen::VectorXd evader_state_;
      Eigen::VectorXd pursuer_total_state_;
      Eigen::VectorXd evader_total_state_;
      std::vector<Eigen::VectorXd>u_pursuer_enum_;
      std::vector<Eigen::VectorXd>u_evader_enum_;
      Eigen::VectorXd target_;

      // since the controllers are 2d, maintain a constant height measurement
      // to prevent z-drift 
      double pursuer_height_ = 0;
      double evader_height_ = 0;

      Student_game_engine_visualizer vis_;

      // functor via std::bind
      costFunction pursuer_cost_functor_ = std::bind(&game_engine::AssetGamesProtocol::jCostPursuer,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5);
      costFunction evader_cost_functor_ = std::bind(&game_engine::AssetGamesProtocol::jCostEvader,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4,std::placeholders::_5);

      dynFunction pursuer_dyn_functor_ = std::bind(&game_engine::AssetGamesProtocol::fDynPursuer,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4);
      dynFunction evader_dyn_functor_ = std::bind(&game_engine::AssetGamesProtocol::fDynEvader,this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3,std::placeholders::_4);

      //~~~ member functions ~~~//

    public:
  
      // cost functions are public because they have to be passed to the costCalculators via functors
        void jCostPursuer( double& cost_total,
            const Eigen::VectorXd& x_self, 
            const Eigen::VectorXd& u_self, 
            const Eigen::VectorXd& x_other,
            const Eigen::VectorXd& u_other);
        void jCostEvader( double& cost_total,
            const Eigen::VectorXd& x_self, 
            const Eigen::VectorXd& u_self, 
            const Eigen::VectorXd& x_other,
            const Eigen::VectorXd& u_other);

    private:

      // generate a trajectory until dt_ time from a constant acceleration over the timeframe
      void trajectoryConstAcc(TrajectoryVector3D& traj, const Eigen::Vector3d& acc,Eigen::Matrix<double,6,1> state, Role role);

      // generate acceleration vectors of constant acceleration pointing around the unit circle
      std::vector<Eigen::VectorXd> constAccCtrlInput(double acc,int num);

      // generate acceleration vectors of constant acceleration pointing around the unit circle and an additional zero vector
      std::vector<Eigen::VectorXd> constAccCtrlInputAndZero(double acc,int num);

      // define the cost penalty of approaching the map bounds
      void costBounds( double& cost_total,
          const Eigen::VectorXd& pos);

    public:
        void fDynPursuer(Eigen::VectorXd& x_1, 
            const Eigen::VectorXd& x_0, 
            const Eigen::VectorXd& u, 
            const double& dt) ;
        void fDynEvader(Eigen::VectorXd& x_1, 
            const Eigen::VectorXd& x_0, 
            const Eigen::VectorXd& u, 
            const double& dt) ;

  };
}
