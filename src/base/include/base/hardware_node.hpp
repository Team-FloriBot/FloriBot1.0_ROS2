#ifndef HARDWARE_NODE_HPP
#define HARDWARE_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "base/msg/wheel_velocities.hpp"
#include "base/diff_drive_lib.hpp"

class HardwareNode : public rclcpp::Node {
public:
    HardwareNode();
private:
    void control_loop();
    void command_callback(const base::msg::WheelVelocities::SharedPtr msg);

    // Hardware Komponenten aus der Library
    std::unique_ptr<SSC32Driver> motor_driver_;
    std::unique_ptr<PhidgetEncoderWrapper> enc_left_, enc_right_;
    std::unique_ptr<PIDController> pid_left_, pid_right_;

    // ROS Schnittstellen
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<base::msg::WheelVelocities>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;

    // Datenhaltung
    std::mutex mtx_;
    double target_l_ = 0.0, target_r_ = 0.0;
    double last_pos_l_ = 0.0, last_pos_r_ = 0.0;
};

#endif
