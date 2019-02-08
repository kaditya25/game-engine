// Author: Tucker Haydon

#ifndef PATH_PLANNING_INTEGRATION_TIME_SPAN_H
#define PATH_PLANNING_INTEGRATION_TIME_SPAN_H

#include <algorithm>
#include <cassert>

namespace path_planning {
  /*
   * POD structure containing time span information used during integrator
   */
  struct TimeSpan {
    const double t0_;
    const double tf_;
    const double dt_;

    TimeSpan(const double t0,
             const double tf,
             const double dt = 0.0) 
      : t0_(t0),
        tf_(tf),
        dt_(std::max(tf - t0, dt)) {
      assert(tf_ > t0_);
      assert(dt_ > 0.0);
      assert(dt_ <= tf_ - t0_);
    }
  };
}

#endif
