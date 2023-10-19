#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt16.h>


class VelocitySensorNode {
   public:
    VelocitySensorNode();
    void run();

   private:
    void motorPositionCallback(const std_msgs::UInt16::ConstPtr &msg);

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Rate loop_rate_;

    uint16_t last_encoder_value_;
};
