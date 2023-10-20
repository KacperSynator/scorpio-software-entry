#include "velocity_controller_node.hpp"

#include <std_msgs/Int8.h>

#include <string_view>

namespace {
using control_toolbox::Pid;

constexpr std::string_view NODE_NAME{"velocity_controler"};
constexpr std::string_view SUB_GOAL_TOPIC{"/virtual_dc_motor_controller/set_velocity_goal"};
constexpr std::string_view SUB_VELOCITY_TOPIC{"/virtual_dc_motor/get_velocity"};
constexpr std::string_view PUB_TOPIC{"/virtual_dc_motor/set_cs"};
constexpr auto QUEUE_SIZE{1};
constexpr auto LOOP_RATE{20};
constexpr auto LOG_LEVEL{ros::console::levels::Debug};

constexpr double p = 0.7;
constexpr double i = 0.2;
constexpr double d = 0.001;
constexpr double i_max = 0.8;
constexpr double i_min = -0.8;

int8_t calculateControlSignal(Pid& pid, const double& error, const ros::Duration& dt) {
    auto command = pid.computeCommand(error, dt);
    ROS_DEBUG("command: %f", command);

    auto cs = static_cast<int8_t>(std::max(-100.0, std::min(100.0, command)));
    ROS_DEBUG("CS: %d", cs);

    return cs;
}
}  // namespace

VelocityContollerNode::VelocityContollerNode() : loop_rate_{LOOP_RATE}, velocity_goal_{}, last_time_{ros::Time::now()} {
    nh_ = ros::NodeHandle(NODE_NAME.data());
    sub_goal_ = nh_.subscribe(SUB_GOAL_TOPIC.data(), QUEUE_SIZE, &VelocityContollerNode::velocityGoalCallback, this);
    sub_velocity_ =
        nh_.subscribe(SUB_VELOCITY_TOPIC.data(), QUEUE_SIZE, &VelocityContollerNode::currentVelocityCallback, this);
    pub_ = nh_.advertise<std_msgs::Int8>(PUB_TOPIC.data(), QUEUE_SIZE);

    pid_.initPid(p, i, d, i_max, i_min);
}

void VelocityContollerNode::velocityGoalCallback(const std_msgs::Float32::ConstPtr& msg) {
    ROS_DEBUG("Received motor velocity goal: %f", msg->data);
    velocity_goal_ = msg->data;
}

void VelocityContollerNode::currentVelocityCallback(const std_msgs::Float32::ConstPtr& msg) {
    ROS_DEBUG("Received current motor velocity: %f", msg->data);

    auto current_time = ros::Time::now();
    std_msgs::Int8 pub_msg{};
    pub_msg.data = calculateControlSignal(pid_, velocity_goal_ - msg->data, current_time - last_time_);
    pub_.publish(pub_msg);
    last_time_ = current_time;
}

void VelocityContollerNode::run() {
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate_.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME.data());
    VelocityContollerNode node;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, LOG_LEVEL);
    node.run();
    return 0;
}
