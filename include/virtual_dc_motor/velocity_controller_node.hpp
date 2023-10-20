#pragma once

#include <control_toolbox/pid.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>


class VelocityContollerNode {
   public:
    VelocityContollerNode();
    void run();

   private:
    void velocityGoalCallback(const std_msgs::Float32::ConstPtr &msg);
    void currentVelocityCallback(const std_msgs::Float32::ConstPtr &msg);

    ros::NodeHandle nh_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_velocity_;
    ros::Publisher pub_;
    ros::Rate loop_rate_;

    ros::Time last_time_;
    float velocity_goal_;

    control_toolbox::Pid pid_;
};
