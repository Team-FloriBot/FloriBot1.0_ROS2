#include "base/hardware_node.hpp"

HardwareNode::HardwareNode() : Node("hardware_node") {
    // Parameter deklarieren
    this->declare_parameter("serial_port", "/dev/ttyS1");
    this->declare_parameter("left_enc_serial", 101902);
    this->declare_parameter("right_enc_serial", 102191);

    // Hardware initialisieren
    motor_driver_ = std::make_unique<SSC32Driver>(this->get_parameter("serial_port").as_string(), 115200); // richtige Baudrate????
    try {
    // Diese Zeilen blockieren nun bis zu 10s, bis die Hardware wirklich da ist
      const int left_sn  = this->get_parameter("left_enc_serial").as_int();
      const int right_sn = this->get_parameter("right_enc_serial").as_int();
      enc_left_  = std::make_unique<PhidgetEncoderWrapper>(left_sn);
      enc_right_ = std::make_unique<PhidgetEncoderWrapper>(right_sn);
      RCLCPP_INFO(this->get_logger(), "Beide Encoder erfolgreich initialisiert.");
    }   
    catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Initialisierungsfehler: %s", e.what());
    }
    
    pid_left_ = std::make_unique<PIDController>(10.0, 0.5, 0.1);
    pid_right_ = std::make_unique<PIDController>(10.0, 0.5, 0.1);

    // ROS 2 Kommunikation
    sub_ = this->create_subscription<base::msg::WheelVelocities>(
        "/wheel_commands", 10, std::bind(&HardwareNode::command_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/wheel_states", 10);
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&HardwareNode::control_loop, this));
}

void HardwareNode::command_callback(const base::msg::WheelVelocities::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mtx_);
    target_l_ = msg->left;
    target_r_ = msg->right;
}

void HardwareNode::control_loop() {
    double dt = 0.01;
    double ticks_to_rad = (2.0 * M_PI) / 1440.0; // Beispielwert

    // 1. Read
    double curr_pos_l = enc_left_->get_position() * ticks_to_rad;
    double curr_pos_r = enc_right_->get_position() * ticks_to_rad;
    double vel_l = (curr_pos_l - last_pos_l_) / dt;
    double vel_r = (curr_pos_r - last_pos_r_) / dt;

    // 2. Control
    double out_l, out_r;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        out_l = pid_left_->compute(target_l_, vel_l, dt);
        out_r = pid_right_->compute(target_r_, vel_r, dt);
    }

    // 3. Write (Mapping auf PWM)
    motor_driver_->send_commands(1500 + (out_l * 500), 1500 + (out_r * 500));

    // 4. Publish
    auto state = sensor_msgs::msg::JointState();
    state.header.stamp = this->now();
    state.name = {"left_wheel", "right_wheel"};
    state.position = {curr_pos_l, curr_pos_r};
    state.velocity = {vel_l, vel_r};
    pub_->publish(state);

    last_pos_l_ = curr_pos_l;
    last_pos_r_ = curr_pos_r;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HardwareNode>());
    rclcpp::shutdown();
    return 0;
}
