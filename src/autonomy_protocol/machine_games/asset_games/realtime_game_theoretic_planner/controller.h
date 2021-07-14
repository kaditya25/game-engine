#ifndef CONTROLLER_H
#define CONTROLLER_H
# include <Eigen/Core>
// abstract class that provides the interface of a discrete time pursuer controller
//  child classes are expected to provide implementation of 
//  calc_control_input() which returns the control input of the controller
//  reset_next_timestep() simply resets cumulative calculations, flags, etc. 
//    child classes are expected to take references to states, so state, time, 
//
class Controller
{
  public:
    virtual Eigen::VectorXd calcControlInput() = 0;

    virtual void resetNextTimeStep() = 0;
};

#endif
