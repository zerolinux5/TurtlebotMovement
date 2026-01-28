#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"


class Movement: public rclcpp::Node
{
    public:
    Movement()
    : Node("movement"), count_(0)
    {
        joy_publisher_ = this->create_publisher<std_msgs::msg::String>("/debug/joy_received", 10);
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, [this](sensor_msgs::msg::Joy::SharedPtr msg){ receiveFromController(msg); }
        );
    }
    private:
    void receiveFromController(sensor_msgs::msg::Joy::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard from Axes[0]: '%f'", msg->axes[0]);
        RCLCPP_INFO(this->get_logger(), "I heard from Axes[1]: '%f'", msg->axes[1]);
    };
    void sendHello() {
        auto message = std_msgs::msg::String();
        message.data = "Hello, " + std::to_string(this->count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        this->joy_publisher_->publish(message);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr joy_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    size_t count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Movement>());
    rclcpp::shutdown();
    return 0;
}