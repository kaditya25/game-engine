// Author: Dan LaChapelle

#pragma once

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>

#include "quad_state.h"
#include "publisher_guard.h"
#include "goal_status.h"

#include "mg_msgs/GoalStatus.h"

namespace game_engine {
  class GoalStatusPublisherNode {
    private:
      // A publisher guard ensures that the Publish() function may be called in
      // a thread-safe manner
      std::shared_ptr<PublisherGuard<mg_msgs::GoalStatus>> publisher_guard_;
      
    public:
      // Constructor.
      GoalStatusPublisherNode(const std::string& topic);

      // Publishes the message
      void Publish(const GoalStatus& goal_status);
  };
}
