#ifndef POTENTIAL_FIELD_H
#define POTENTIAL_FIELD_H

#include <cmath>

#include "types.h"
#include "map3d.h"

namespace game_engine {
    class PotentialField {
        // protected instead of private so that child Class can access
    protected:
        struct Options {
            // eta: the termination condition for gradient descent. Also the descent direction
            double eta = 1.0;
            // zeta: the attractive factor multiplier
            double zeta = 10.0;
            // ada: the repulsive factor multiplier
            double ada = 0.1;
            // d_thresh: threshold for transition between quadratic and conic potential
            double d_thresh = 2.0;
            // q_thresh: range of influence of obstacle
            double q_thresh = 0.1;

            bool terminate = false;

            Options() {}
        };

        Options options_;
    public:
        PotentialField(const Map3D& map,
                       const Options& options = Options())
                      : options_(options) {};

        // Calculates Attractive Gradient towards goal
        Eigen::Vector3d AttractiveGradient(const Eigen::Vector3d& current_position, const Eigen::Vector3d& goal_position);

        // Calculates Repulsive Gradient (cumulative) based on all obstacles
        Eigen::Vector3d RepulsiveGradient(const Eigen::Vector3d& current_position, const Map3D& map);

        // Returns the multi-dimensional norm of our gradient
        double MultiDimNorm(const std::vector<double> partials);

        // Perform Gradient Descent for one step to move closer to the goal
        Eigen::Vector3d OneStepGradientDescent(const Eigen::Vector3d& current_position, const Eigen::Vector3d& goal_position, const Map3D& map);

        // Returns whether we have reached the goal
        bool ReturnTerminate();
    };
}
#endif