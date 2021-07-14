#include "vel_match_controller.h"

Eigen::VectorXd velMatchController::calcControlInput()
{
  auto vel_k1 = velMatchAlg();
  Eigen::VectorXd accel(2);
  accel = (vel_k1-x_.tail(2))/dt_;
  // need to normalize acceleration when 
  // opponent velocity is non constant
  if(accel.norm()>accel_max_)
    accel = accel.normalized()*accel_max_;
  return accel;
}

// also see matlab implementation which has been tested a bunch
Eigen::Vector2d velMatchController::velMatchAlg()
{
  // algorithm calculates desired velocity at next timestep
  Eigen::Vector2d vel_k1;

  double alpha_a;
  double alpha_a_decel;
  double alpha_v;
  double alpha_p;
  double alpha;

  Eigen::Vector2d vel_t = x_opponent_.tail(2);
  Eigen::Vector2d vel_k = x_.tail(2);

  // r is the relative position vector
  Eigen::Vector2d r_vec = x_opponent_.head(2)-x_.head(2);
  // velocity difference vector for clarity

  // case for being an evader: want to move in the opposite direction
  if (role_== Role::evader)
  {
    vel_t = vel_t;
    r_vec = -r_vec;
    // evader needs to approach target, so include a
    // portion of the distance to target in r_vec.
    // scaling by the opposite distance implies that the 
    // evader determines importance of target/pursuer
    // by their relative distance
    Eigen::Vector2d t_vec = x_target_-x_.head(2);
    r_vec = r_vec + t_vec;
    double dist_total = r_vec.norm()+ t_vec.norm();
    r_vec = t_vec.norm()/dist_total * r_vec +
            r_vec.norm()/dist_total * t_vec ;
  }

  Eigen::Vector2d vel_diff = vel_t-vel_k;

  if (x_opponent_.tail(2).norm()>=vel_max_)
  {
    // no great way to handle this case
    vel_t = vel_t.normalized()*vel_max_;
    //return vel_k1;
  }
  if (r_vec.norm() <= intercept_threshold_)
  { // officially intercepted opponent
    vel_k1 = vel_t; 
    return vel_k1;
  }


  // check acceleration limit by solving 
  // the quadratic representing the interception 
  // of the rendezvous line and the circle 
  // of allowable accelerations
  double a = r_vec.squaredNorm();
  double b = 2*vel_diff.dot(r_vec);
  double c = vel_diff.squaredNorm() - std::pow(accel_max_*dt_,2);
  double det = b*b-4*a*c;

  if (det<0)
  { // cannot accelerate high enough within the time window
    // to match opponent velocity. 
    // Not actually acknowledged in the paper, so
    // imlementation simply max accelerates perpendicular to r
    Eigen::Vector2d vel_dir = {-r_vec[1],r_vec[0]};
    if (vel_dir.dot(vel_t)<0)
    {
      vel_dir = -vel_dir;
    }
    vel_k1 = vel_k + vel_dir.normalized()*accel_max_*dt_;
    if (vel_k1.norm() > vel_max_)
      vel_k1 = vel_k1.normalized()*vel_max_;
    return vel_k1;
  }
  else
  { // determine accel limited velocity
    alpha_a = (-b+sqrt(det))/(2*a);
    alpha_a_decel = (-b-sqrt(det))/(2*a);
  }

  // check velocity limit by doing the same with the
  // circle of max velocity, and the laziness of using
  // the same variables for the quadratic
  a = r_vec.squaredNorm(); // same as before
  b = 2*vel_t.dot(r_vec);
  c = vel_t.squaredNorm() - vel_max_*vel_max_;
  det = b*b-4*a*c;

  if (det<0)
  {
    // cannot achieve vel_opponent within velocity envelope
    // should have been caught before
    std::cerr << "vel match error: velocity limit is unsolvable!";
    return vel_k1;
  }
  else 
  { // determine vel limited velocity
    alpha_v = (-b+sqrt(det))/(2*a);
  }

  // there SHOULD always be a position limit
  //    the exception is when position can be matched 
  //    but velocity cannot; i.e. overshoot. in this 
  //    case maximally decelerate and deal with it
  //    after point of overshoot
  a = 1;
  b = accel_max_*dt_;
  c = vel_k.dot(r_vec.normalized())*accel_max_*dt_ 
        - 2*accel_max_*r_vec.norm();
  det = b*b-4*a*c;
  
  if (det<0)
  {
    // cannot match position and velocity at the same time
    alpha_p = alpha_a_decel;
  }
  else
  {
    alpha_p = (-b+sqrt(det))/(2*a)/r_vec.norm();
  }
  // select the minimum alpha
  alpha = std::min(std::min(alpha_a,alpha_v),alpha_p);

  vel_k1 = vel_t + r_vec*alpha;
  return vel_k1;
}

void velMatchController::resetNextTimeStep()
{// nothing needs to happen on the controller side
  return;
}
