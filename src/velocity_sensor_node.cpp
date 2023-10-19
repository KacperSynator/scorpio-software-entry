#include "velocity_sensor_node.hpp"

#include <std_msgs/Float32.h>

#include <string_view>

namespace {
constexpr std::string_view NODE_NAME{"velocity_sensor"};
constexpr std::string_view SUB_TOPIC{"/virtual_dc_motor/get_position"};
constexpr std::string_view PUB_TOPIC{"/virtual_dc_motor/get_velocity"};
constexpr auto QUEUE_SIZE{10};
constexpr auto LOOP_RATE{400};
constexpr auto LOG_LEVEL{ros::console::levels::Debug};
constexpr auto ENCODER_RANGE{4095};
constexpr auto SEC_IN_MIN{60};

float calculateVelocity(const uint16_t& current_encoder_value, const uint16_t& last_encoder_value) {
    auto angle_diff = static_cast<int16_t>(current_encoder_value) - static_cast<int16_t>(last_encoder_value);
   
    if (angle_diff < -3000) {
        angle_diff += ENCODER_RANGE;
    }

    if (angle_diff > 3000) {
        angle_diff -= ENCODER_RANGE;
    }
    
    return static_cast<float>(angle_diff) / ENCODER_RANGE * LOOP_RATE * SEC_IN_MIN;
}
}  // namespace

VelocitySensorNode::VelocitySensorNode() : loop_rate_{LOOP_RATE}, last_encoder_value_{} {
    nh_ = ros::NodeHandle(NODE_NAME.data());
    sub_ = nh_.subscribe(SUB_TOPIC.data(), QUEUE_SIZE, &VelocitySensorNode::motorPositionCallback, this);
    pub_ = nh_.advertise<std_msgs::Float32>(PUB_TOPIC.data(), QUEUE_SIZE);
}

void VelocitySensorNode::motorPositionCallback(const std_msgs::UInt16::ConstPtr& msg) {
    ROS_DEBUG("Received current motor position: %d", msg->data);

    auto current_encoder_value = msg->data;
    const auto velocity = calculateVelocity(current_encoder_value, last_encoder_value_);

    ROS_DEBUG("Sending velocity: %f", velocity);

    std_msgs::Float32 pub_msg{};
    pub_msg.data = velocity;
    pub_.publish(pub_msg);

    last_encoder_value_ = current_encoder_value;
}

void VelocitySensorNode::run() {
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate_.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_NAME.data());
    VelocitySensorNode node;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, LOG_LEVEL);
    node.run();
    return 0;
}
