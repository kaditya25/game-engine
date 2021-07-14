#ifndef VEL_MATCH_CONTROLLER_H
#define VEL_MATCH_CONTROLLER_H
#include "helper.h"
#include "controller.h"

enum Role : int
{
  pursuer = 1,
  evader = -1,

};

class velMatchController : public Controller
{
  public:

    double intercept_threshold_ = 0.1; // minimum distance considered to have intercepted the opponent

    velMatchController(
        Eigen::VectorXd& x_0,
        Eigen::VectorXd& x_t_0,
        Eigen::VectorXd& x_target,
        const double& a_max,
        const double& v_max,
        const double& dt,
        const Role role=pursuer     ):
      x_(x_0),
      x_opponent_(x_t_0),
      x_target_(x_target),
      accel_max_(a_max),
      vel_max_(v_max),
      dt_(dt),
      role_(role)    {}

    Eigen::VectorXd calcControlInput() override;

    void resetNextTimeStep() override;

  private:

    const Role role_;

    Eigen::VectorXd& x_; // state is expected to be [px,py,vx,vy]^T
    Eigen::VectorXd& x_opponent_; // state is expected to be [px,py,vx,vy]^T
    Eigen::VectorXd& x_target_; // target is assumed to be a stationary 2d point
    // dynamics are not necessary, since velocity matching assumes pure second integrator dynamics
    //int& K_i_;
    const double& dt_;
    const double accel_max_;
    const double vel_max_;

// does the actual algorithm's work, calculating desired 
// velocity at next step, assuming constant acceleration
    Eigen::Vector2d velMatchAlg();

};

#endif
