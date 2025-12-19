#ifndef HARDWARE_NODE_HPP
#define HARDWARE_NODE_HPP

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "base/msg/wheel_velocities.hpp"
#include "base/diff_drive_lib.hpp"

class HardwareNode : public rclcpp::Node
{
public:
    HardwareNode();

private:
    void command_callback(
        const base::msg::WheelVelocities::SharedPtr msg);
    void control_loop();

    // ROS
    rclcpp::Subscription<base::msg::WheelVelocities>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Hardware
    std::unique_ptr<SSC32Driver> motor_driver_;
    std::unique_ptr<PhidgetEncoderWrapper> enc_left_;
    std::unique_ptr<PhidgetEncoderWrapper> enc_right_;

    // Regler
    std::unique_ptr<PIDController> pid_left_;
    std::unique_ptr<PIDController> pid_right_;

    // State
    double target_l_{0.0};
    double target_r_{0.0};
    double last_pos_l_{0.0};
    double last_pos_r_{0.0};

    rclcpp::Time last_time_;
    std::mutex mtx_;
};

#endif  // HARDWARE_NODE_HPP

