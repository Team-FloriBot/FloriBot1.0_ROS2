#include "base/sensor_aggregator_node.h"

using std::placeholders::_1;

namespace base {

SensorAggregatorNode::SensorAggregatorNode() : Node("sensor_aggregator_node") {
    // Publisher auf dem Topic, auf das KinematicsNode wartet
    pub_wheel_states_ = this->create_publisher<base::msg::WheelVelocities>("/wheel_states", 10);

    // Abonnements für die vier individuellen Controller-Status-Topics
    sub_left_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_left/state", 10, std::bind(&SensorAggregatorNode::leftStateCallback, this, _1));
    sub_right_ = this->create_subscription<base::msg::WheelControllerState>(
        "/wheel_controller_right/state", 10, std::bind(&SensorAggregatorNode::rightStateCallback, this, _1));

    // Timer, um die gesammelten Daten regelmäßig (z.B. 50Hz) zu veröffentlichen
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20), 
        std::bind(&SensorAggregatorNode::publishAggregatedStates, this));

    RCLCPP_INFO(this->get_logger(), "Sensor Aggregator Node Initialized.");
}

void SensorAggregatorNode::leftStateCallback(const base::msg::WheelControllerState::SharedPtr msg) {
    current_velocities_.left = msg->measured_velocity;
}
void SensorAggregatorNode::rightStateCallback(const base::msg::WheelControllerState::SharedPtr msg) {
    current_velocities_.right = msg->measured_velocity;
}

void SensorAggregatorNode::publishAggregatedStates() {
    // Veröffentliche die gesammelten und aktuellsten Daten
    pub_wheel_states_->publish(current_velocities_);
}

} // namespace base

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<base::SensorAggregatorNode>());
    rclcpp::shutdown();
    return 0;
}
