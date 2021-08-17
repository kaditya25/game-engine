#ifndef BALLOON_POSITION_SUBSCRIBER_NODE_H
#define BALLOON_POSITION_SUBSCRIBER_NODE_H

#include <ros/ros.h>
#include <memory>
#include <string>
#include <functional>
#include <Eigen/StdVector>
#include <geometry_msgs/Point.h>

namespace game_engine {
    class BalloonPositionSubscriberNode {
    private:
        // ROS node handle
        ros::NodeHandle node_handle_;

        // ROS subscriber
        ros::Subscriber subscriber_;

        // Subscriber callback function. Converts ROS message into local
        // Eigen::Vector3d message

        void SubscriberCallback(const geometry_msgs::Point& msg);

    public:
        // Pointer to balloon status. No guarantees on read/write threading access
        std::shared_ptr<Eigen::Vector3d> balloon_position_;
        // Constructor.
        //
        // Note parameters are intentionally copied.
        BalloonPositionSubscriberNode(
            const std::string& topic,
            std::shared_ptr<Eigen::Vector3d> balloon_position
        );
    };
}

#endif