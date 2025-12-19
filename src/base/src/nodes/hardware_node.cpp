#include "base/hardware_node.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

HardwareNode::HardwareNode()
: Node("hardware_node")
{
    this->declare_parameter("serial_port", "/dev/ttyS1");
    this->declare_parameter("left_enc_serial", 101902);
    this->declare_parameter("right_enc_serial", 102191);

    const std::string serial_port =
        this->get_parameter("serial_port").as_string();
    const int left_sn =
        this->get_parameter("left_enc_serial").as_int();
    const int right_sn =
        this->get_parameter("right_enc_serial").as_int();

    try {
        motor_driver_ =
            std::make_unique<SSC32Driver>(serial_port, 115200);

        enc_left_  = std::make_unique<PhidgetEncoderWrapper>(left_sn);
        enc_right_ = std::make_unique<PhidgetEncoderWrapper>(right_sn);

        RCLCPP_INFO(
            this->get_logger(),
            "Hardware initialisiert (SSC32 + Encoder %d / %d)",
            left_sn, right_sn
        );
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(
            this->get_logger(),
            "Hardware-Initialisierung fehlgeschlagen: %s",
            e.what()
        );
        throw;
    }

    pid_left_  = std::make_unique<PIDController>(10.0, 0.5, 0.1);
    pid_right_ = std::make_unique<PIDController>(10.0, 0.5, 0.1);

    sub_ = this->create_subscription<base::msg::WheelVelocities>(
        "/wheel_commands",
        10,
        std::bind(&HardwareNode::command_callback, this, std::placeholders::_1)
    );

    pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/wheel_states",
        10
    );

    last_time_ = this->now();
    timer_ = this->create_wall_timer(
        10ms,
        std::bind(&HardwareNode::control_loop, this)
    );
}

void HardwareNode::command_callback(
    const base::msg::WheelVelocities::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mtx_);
    target_l_ = msg->left;
    target_r_ = msg->right;
}

void HardwareNode::control_loop()
{
    if (!motor_driver_ || !enc_left_ || !enc_right_) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "Hardware nicht bereit – control_loop übersprungen"
        );
        return;
    }

    const rclcpp::Time now = this->now();
    const double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt <= 0.0) return;

    constexpr double ticks_to_rad =
        (2.0 * M_PI) / 1440.0;

    const double curr_pos_l =
        enc_left_->get_position() * ticks_to_rad;
    const double curr_pos_r =
        enc_right_->get_position() * ticks_to_rad;

    const double vel_l =
        (curr_pos_l - last_pos_l_) / dt;
    const double vel_r =
        (curr_pos_r - last_pos_r_) / dt;

    double target_l;
    double target_r;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        target_l = target_l_;
        target_r = target_r_;
    }

    const double out_l =
        pid_left_->compute(target_l, vel_l, dt);
    const double out_r =
        pid_right_->compute(target_r, vel_r, dt);

    const int pwm_left  = 1500 + static_cast<int>(out_l * 500.0);
    const int pwm_right = 1500 + static_cast<int>(out_r * 500.0);

    motor_driver_->send_commands(pwm_left, pwm_right);

    sensor_msgs::msg::JointState state;
    state.header.stamp = now;
    state.name = {"left_wheel", "right_wheel"};
    state.position = {curr_pos_l, curr_pos_r};
    state.velocity = {vel_l, vel_r};

    pub_->publish(state);

    last_pos_l_ = curr_pos_l;
    last_pos_r_ = curr_pos_r;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareNode>());
    rclcpp::shutdown();
    return 0;
}


