#include <rclcpp/rclcpp.hpp>
#include <base/msg/wheel_velocities.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class SabertoothNode : public rclcpp::Node {
public:
    SabertoothNode() : Node("sabertooth_node") {
        // Parameter (Standardwerte für PC-USB-Adapter)
        this->declare_parameter("port", "/dev/ttyS1");
        this->declare_parameter("baud", 9600);
        this->declare_parameter("address", 128);

        std::string port = this->get_parameter("port").as_string();
        int baud = this->get_parameter("baud").as_int();

        fd_ = open(port.c_str(), O_RDWR | O_NOCTTY);
        if (fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "Fehler: Port %s konnte nicht geöffnet werden!", port.c_str());
        } else {
            setup_serial(baud);
            RCLCPP_INFO(this->get_logger(), "Sabertooth auf %s aktiv.", port.c_str());
        }

        sub_wheel_cmd_ = this->create_subscription<base::msg::WheelVelocities>(
            "/wheel_commands", 10, std::bind(&SabertoothNode::commandCallback, this, std::placeholders::_1));
    }

    ~SabertoothNode() { stop_motors(); if (fd_ != -1) close(fd_); }

private:
    int fd_;
    rclcpp::Subscription<base::msg::WheelVelocities>::SharedPtr sub_wheel_cmd_;

    void setup_serial(int baudrate) {
        struct termios options;
        tcgetattr(fd_, &options);
        cfsetispeed(&options, B9600); 
        cfsetospeed(&options, B9600);
        options.c_cflag |= (CLOCAL | CREAD | CS8);
        options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        tcsetattr(fd_, TCSANOW, &options);
    }

    void send_packet(uint8_t command, uint8_t value) {
        if (fd_ == -1) return;
        uint8_t address = this->get_parameter("address").as_int();
        uint8_t checksum = (address + command + value) & 127;
        uint8_t packet[4] = {address, command, value, checksum};
        write(fd_, packet, 4);
    }

    void commandCallback(const base::msg::WheelVelocities::SharedPtr msg) {
        uint8_t power = 30; // 30/127 entspricht ca. 23% Leistung für den Test

        // Einfache Logik: Wenn ein Befehl kommt, gib konstante Leistung
        if (std::abs(msg->left) > 0.05 || std::abs(msg->right) > 0.05) {
            send_packet(0, power); // Motor 1 (Links) Vorwärts
            send_packet(4, power); // Motor 2 (Rechts) Vorwärts
        } else {
            stop_motors();
        }
    }

    void stop_motors() {
        send_packet(0, 0);
        send_packet(4, 0);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SabertoothNode>());
    rclcpp::shutdown();
    return 0;
}
