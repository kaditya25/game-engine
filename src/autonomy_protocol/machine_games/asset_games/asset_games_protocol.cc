#include "asset_games_protocol.h"

namespace game_engine {

  // dynamics ignores z
  // point mass double integrator dynamics 
  void AssetGamesProtocol::fDynPursuer(Eigen::VectorXd& x_1, 
      const Eigen::VectorXd& x_0, 
      const Eigen::VectorXd& u, 
      const double& dt) 
  {
    Eigen::MatrixXd A(4,4);
    A << Eigen::MatrixXd::Identity(2,2), Eigen::MatrixXd::Identity(2,2)*dt,
      Eigen::MatrixXd::Zero(2,2), Eigen::MatrixXd::Identity(2,2);
    Eigen::MatrixXd B(4,2);
    B << Eigen::MatrixXd::Identity(2,2)*0.5*std::pow(dt,2), Eigen::MatrixXd::Identity(2,2)*dt;
    x_1 = A*x_0 + B*u;
    return;
  }

  void AssetGamesProtocol::fDynEvader(Eigen::VectorXd& x_1, 
      const Eigen::VectorXd& x_0, 
      const Eigen::VectorXd& u, 
      const double& dt) 
  {
    Eigen::MatrixXd A(4,4);
    A << Eigen::MatrixXd::Identity(2,2), Eigen::MatrixXd::Identity(2,2)*dt,
      Eigen::MatrixXd::Zero(2,2), Eigen::MatrixXd::Identity(2,2);
    Eigen::MatrixXd B(4,2);
    B << Eigen::MatrixXd::Identity(2,2)*0.5*std::pow(dt,2), Eigen::MatrixXd::Identity(2,2)*dt;
    x_1 = A*x_0 + B*u;
    return;
  }
  void AssetGamesProtocol::costBounds( double& cost_total, const Eigen::VectorXd& pos ) {
    Eigen::Vector3d nearest_point = map3d_.ClosestPoint(pos);
    double dist = (nearest_point-pos).norm() ;
    // penalty scales with distance
    if (dist < obstacle_crit_dist_ && nearest_point[2] == pos[2])
    {
      cost_total += obstacle_gain_ * 
                std::pow((1/dist - 1/obstacle_crit_dist_),2);
    }
    // map3d::IsFreeSpace should check Contains, but does not
    if (! map3d_.Contains( pos ) || ! map3d_.IsFreeSpace( pos ) )
    {
      cost_total += 20000;
    }
  }

  void AssetGamesProtocol::jCostEvader( double& cost_total,
      const Eigen::VectorXd& x_self, 
      const Eigen::VectorXd& u_self, 
      const Eigen::VectorXd& x_other,
      const Eigen::VectorXd& u_other)
  {
    double acc_max = acc_max_evader_+0.01;
    double vel_max = vel_max_evader_+0.01;
    Eigen::DiagonalMatrix<double,4> Q_pur;
    Q_pur.diagonal() << 5,5,0,0;
    Eigen::DiagonalMatrix<double,2> R_pur;
    R_pur.diagonal() << 1,1;
    Eigen::DiagonalMatrix<double,2> Q_target;
    Q_target.diagonal() << 5,5;
    Eigen::VectorXd e = x_self-x_other;
    Eigen::VectorXd e_target = x_self.head(2)-target_;

      // cumulative cost calculation
      //cost_total += -(double)(e.transpose()*Q_pur*e) *e_target.dot(e_target);
      //cost_total += (double)(e_target.transpose()*Q_target*e_target) *e.head(2).dot(e.head(2));
      cost_total += -(double)(e.transpose()*Q_pur*e) ;
      cost_total += (double)(e_target.transpose()*Q_target*e_target) ;

    // end condition: if target has been reached
    if (e_target.norm() < 1)
    { 
      cost_total += -2000/ (e_target.norm()+.1 );
      cost_total += -2000* ( x_self.tail<2>().norm() );
    }
    // end condition: if evader has been caughtk
    if (e.norm() < 1.25)
    { cost_total += 1000/ (e.norm()+.1); }


    //since this is only 2d, going to use an artificial height for testing
    double test_height = -4;
    Eigen::Vector3d pos; pos << x_self.head<2>(),test_height;

    // soft limits on acceleration and velocity 
    if (u_self.norm()>acc_max)
      cost_total += 20000;
    if (x_self.tail(2).norm()>vel_max)
      cost_total += 20000;

    costBounds(cost_total,pos);

    return;
  }

  void AssetGamesProtocol::jCostPursuer( double& cost_total,
      const Eigen::VectorXd& x_self, 
      const Eigen::VectorXd& u_self, 
      const Eigen::VectorXd& x_other,
      const Eigen::VectorXd& u_other)
  {
    double acc_max = acc_max_pursuer_+0.01;
    double vel_max = vel_max_pursuer_+0.01;
    Eigen::DiagonalMatrix<double,4> Q_pur;
    Q_pur.diagonal() << 5,5,3,3;
    Eigen::DiagonalMatrix<double,2> R_pur;
    R_pur.diagonal() << 1,1;
    Eigen::DiagonalMatrix<double,2> Q_target;
    Q_target.diagonal() << 5,5;
    Eigen::VectorXd e = x_self-x_other;
    Eigen::VectorXd e_target = x_other.head(2)-target_;

    // cumulative cost calculation
    cost_total += (double)(e.transpose()*Q_pur*e) ;
    //cost_total += u_self.transpose()*R_pur*u_self;
    cost_total += (double)(-e_target.transpose()*Q_target*e_target) ;

    // end condition: if target has been reached
    if (e_target.norm() < 1)
    { cost_total += 2000/ (e_target.norm()+.1); }
    // end condition: if evader has been caught
    if (e.norm() < 1.25)
    { cost_total += -1000/ (e.norm()+.1); }

    //since this is only 2d, going to use an artificial height for testing
    double test_height = -4;
    Eigen::Vector3d pos; pos << x_self.head<2>(),test_height;

    // soft limits on acceleration and velocity 
    if (u_self.norm()>acc_max)
      cost_total += 20000;
    if (x_self.tail(2).norm()>vel_max)
      cost_total += 20000;

    // idea: pursuer cannot 'camp' the target
    //if ( (x_self.head<2>()-target_).norm()<1.5 )
    //{ cost_total += 10000; }

    costBounds(cost_total,pos);

    return;
  }

  std::vector<Eigen::VectorXd> AssetGamesProtocol::constAccCtrlInputAndZero(double acc,int num)
  {
    auto vec = constAccCtrlInput(acc,num);
      vec.push_back(Eigen::VectorXd::Zero(3));
    return vec;
  }

  std::vector<Eigen::VectorXd> AssetGamesProtocol::constAccCtrlInput(double acc,int num)
  {
    // enumerate possible control
    std::vector<Eigen::VectorXd> u_enumerated(num);
    for (int i=0; i < num; i++)
    {
      u_enumerated[i].resize(2);
      u_enumerated[i][0] = acc*cos(2*M_PI*i/num);
      u_enumerated[i][1] = acc*sin(2*M_PI*i/num);
    }
    return u_enumerated;
  }

  void AssetGamesProtocol::trajectoryConstAcc( TrajectoryVector3D& trajectory_vector, const Eigen::Vector3d& acc, Eigen::Matrix<double,6,1>state, Role role)
  {
    // hotfix to maintain 2d ness
    state[2] = -4;
    switch (role)
    {
      case evader:
        { state[2] = evader_height_; break; }
      case pursuer:
        { state[2] = pursuer_height_; break; }
    }

    state[5] = 0;
    double yaw = 0;
    auto realtime_now = std::chrono::system_clock::now();
    double sample_period = 0.01; // seconds
    int num_samples = (int)(dt_/sample_period) + 1;
    std::chrono::duration<double> time_sample;

    Eigen::MatrixXd A(6,6);
    A << Eigen::MatrixXd::Identity(3,3), Eigen::MatrixXd::Identity(3,3)*sample_period,
      Eigen::MatrixXd::Zero(3,3), Eigen::MatrixXd::Identity(3,3);
    Eigen::MatrixXd B(6,3);
    B << Eigen::MatrixXd::Identity(3,3)*0.5*std::pow(sample_period,2), Eigen::MatrixXd::Identity(3,3)*sample_period;

    // implement some sort of post processing obstacle avoidance
    double theta = 0;
    Eigen::Vector3d acc_aug = acc;
    Eigen::Vector3d vel_aug = state.tail<3>();
    Eigen::Vector3d pos_final;
    Eigen::Vector3d vel_final;

    Eigen::Transform<double,3,Eigen::Isometry> rotation;
    do
    {
      theta = -theta;
      rotation = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
      acc_aug = rotation * acc;
      vel_aug = rotation * state.tail<3>();
      pos_final = state.head<3>() + Eigen::MatrixXd::Identity(3,3)*dt_ * vel_aug + 
                                    Eigen::MatrixXd::Identity(3,3)*0.5*dt_*dt_ * acc_aug;

    } while (abs(theta+=(theta>=0)*0.01) < 2*M_PI && (! map3d_.Contains( pos_final ) || ! map3d_.IsFreeSpace( pos_final )) );
    if (abs(theta)>0.05)
      std::cout << "course correcting theta is " << theta << "\n";

    state.tail(3) = vel_aug;

    if (acc.dot(vel_aug)>0)
    { vel_final = state.tail<3>() + Eigen::MatrixXd::Identity(3,3)*dt_ * acc_aug; }
    else
    { vel_final = state.tail<3>(); }


    for (int idx=0; idx < num_samples; idx++)
    {
      time_sample = realtime_now.time_since_epoch() + std::chrono::duration<double>(dt_*idx/num_samples);

      trajectory_vector.push_back(
          (Eigen::Matrix<double, 11, 1>() << 
           //state.head<3>(),vel_final, acc_aug, 
           state, acc_aug, 
           yaw,
           time_sample.count()
          ).finished());
      state = A*state + B*(acc_aug);
    }
  }

  void AssetGamesProtocol::trajectoryStationary( TrajectoryVector3D& trajectory_vector, const Eigen::Matrix<double,6,1>& state )
  {
    auto realtime_now = std::chrono::system_clock::now();
    double sample_period = 0.01; // seconds
    int num_samples = (int)(dt_/sample_period) + 1;
    std::chrono::duration<double> time_sample;

    for (int idx=0; idx < num_samples; idx++)
    {
      time_sample = realtime_now.time_since_epoch() + std::chrono::duration<double>(dt_*idx/num_samples);
      trajectory_vector.push_back(
          (Eigen::Matrix<double, 11, 1>() << 
           state.head<3>(),Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 
           0,
           time_sample.count()
          ).finished());
    }
  }

  std::unordered_map<std::string, Trajectory> AssetGamesProtocol::UpdateTrajectories() 
  {
    std::unordered_map<std::string, Trajectory> trajectory_map;

    if (friendly_names_.size()!=2)
    {
      std::cerr << "Warning: expect only two friendly player!\n";
    }
    else if (enemy_names_.size()!=0)
    {
      std::cerr << "Warning: expect only friendlies can be controlled!\n";
    }
    else
    {

      std::string pursuer = friendly_names_[0];
      std::string evader = friendly_names_[1];

      Eigen::Vector3d pos,vel;
      this->snapshot_->Position(pursuer, pos);
      this->snapshot_->Velocity(pursuer, vel);
      pursuer_state_ << pos.head(2),vel.head(2);
      pursuer_total_state_ << pos,vel;
      this->snapshot_->Position(evader, pos);
      this->snapshot_->Velocity(evader, vel);
      evader_state_ << pos.head(2),vel.head(2);
      evader_total_state_ << pos,vel;

      if (pursuer_total_state_.norm()<1e-2 || evader_total_state_.norm()<1e-2)
      {
        std::cout << "received zero states!\n"; 
        return trajectory_map;
      }
      else if (pursuer_height_==0 || evader_height_==0)
      {
        pursuer_height_ = pursuer_total_state_[2];
        evader_height_ = evader_total_state_[2];
        target_ = (*red_balloon_position_).head(2);
        if (!initialized_)
        {
          initialized_ = true;
          t_begin_ = std::chrono::system_clock::now();
        }
      }

      if (stop_flag_) 
      {
        t_end_ = std::chrono::system_clock::now();
        std::chrono::duration<double> runtime = t_end_-t_begin_;
        std::cout << "runtime was " << runtime.count() << "\n";
        this->Stop();
        exit(0);
      }
      
      { std::cout << "mediation code is " <<
          static_cast<int>(trajectoryCodeMap_[evader].code) << std::endl; };

      switch (trajectoryCodeMap_[evader].code) {
        case MediationLayerCode::Success:
          // You probably won't need to do anything in response to Success.
          std::cout << "success\n";
          break;
        case MediationLayerCode::QuadTooCloseToAnotherQuad: 
          std::cout << "QuadTooCloseToAnotherQuad." << std::endl;
          break;
        }

      if (trajectoryCodeMap_[evader].code==MediationLayerCode::QuadTooCloseToAnotherQuad || trajectoryCodeMap_[evader].code==MediationLayerCode::QuadTrajectoryCollidesWithAnotherQuad || trajectoryCodeMap_[evader].code==MediationLayerCode::FailedToCallService)
      {
        std::cout << "evader stopped!\n";
        TrajectoryVector3D trajectory_vector;
        trajectoryStationary(trajectory_vector,pursuer_total_state_);
        Trajectory trajectory(trajectory_vector);
        trajectory_map[pursuer] = trajectory;

        TrajectoryVector3D trajectory_vector2;
        trajectoryStationary(trajectory_vector2,evader_total_state_);
        Trajectory trajectory2(trajectory_vector2);
        trajectory_map[evader] = trajectory2;

        vis_.drawDot(pursuer_total_state_.head<3>(),5678,0);
        vis_.drawDot(evader_total_state_.head<3>(),5679,1);
        stop_flag_ = true;
        return trajectory_map;
      }

      switch (trajectoryCodeMap_[evader].code) {
        case MediationLayerCode::Success:
          // You probably won't need to do anything in response to Success.
          break;
        case MediationLayerCode::QuadTooCloseToAnotherQuad: {
          std::cout << "QuadTooCloseToAnotherQuad." << std::endl;
          break;
        }
        default:
          // If you want to see a numerical MediationLayerCode value, you can cast and
          // print the code as shown below.
          std::cout << "MediationLayerCode: " <<
                    static_cast<int>(trajectoryCodeMap_[evader].code) << std::endl;
      }

      if (red_balloon_status_->popped)
      {
        std::cout << "balloon has been popped!\n";
        TrajectoryVector3D trajectory_vector;
        trajectoryStationary(trajectory_vector,pursuer_total_state_);
        Trajectory trajectory(trajectory_vector);
        trajectory_map[pursuer] = trajectory;

        TrajectoryVector3D trajectory_vector2;
        trajectoryStationary(trajectory_vector2,evader_total_state_);
        Trajectory trajectory2(trajectory_vector2);
        trajectory_map[evader] = trajectory2;

        vis_.drawDot(pursuer_total_state_.head<3>(),5678,0);
        vis_.drawDot(evader_total_state_.head<3>(),5679,1);
        stop_flag_ = true;
        return trajectory_map;
      }
      else 
      {
        // no way to set action to delete!
        vis_.drawDot(Eigen::Vector3d::Zero(),5678,0);
        vis_.drawDot(Eigen::Vector3d::Zero(),5679,1);
      }

      Eigen::Vector2d u;
      Eigen::Vector3d acc;

      // calculate controller output
      auto t_start = std::chrono::system_clock::now();
      std::cout << "pursuer is " << pursuer << "\n";
      u = (controllers_[0]->calcControlInput());
      controllers_[0]->resetNextTimeStep();
      acc << u,0;
      std::cout << "acceleration command: ";
      helper::print(acc.transpose());
      auto t_end = std::chrono::system_clock::now();
      std::chrono::duration<double> diff = t_end-t_start;
      std::cout << "realtime: "<<diff.count() << "\n";

      // create trajectory
      TrajectoryVector3D trajectory_vector;
      trajectoryConstAcc(trajectory_vector,acc,pursuer_total_state_,Role::pursuer);
      Trajectory trajectory(trajectory_vector);
      trajectory_map[pursuer] = trajectory;

      // calculate controller output
      std::cout << "evader is " << evader << "\n";
      t_start = std::chrono::system_clock::now();
      u = (controllers_[1]->calcControlInput());
      controllers_[1]->resetNextTimeStep();
      acc << u,0;
      std::cout << "acceleration command: ";
      helper::print(acc.transpose());
      t_end = std::chrono::system_clock::now();
      diff = t_end-t_start;
      std::cout << "realtime: "<<diff.count() << "\n";

      // create trajectory
      TrajectoryVector3D trajectory_vector2;
      trajectoryConstAcc(trajectory_vector2,acc,evader_total_state_,Role::evader);
      Trajectory trajectory2(trajectory_vector2);
      trajectory_map[evader] = trajectory2;

      // visualize with offsets 0 and 1
      vis_.drawTrajectory(trajectory,0);
      vis_.drawTrajectory(trajectory2,1);
    }


    return trajectory_map;
  }

}
