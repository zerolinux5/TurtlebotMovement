#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

constexpr int ANGULAR = 0;
constexpr int LINEAR = 1;
constexpr float SPEED_OFFSET = 2.0f;

class Movement: public rclcpp::Node
{
    public:
    Movement()
    : Node("movement"), count_(0)
    {
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, [this](sensor_msgs::msg::Joy::SharedPtr msg){ receiveFromController(msg); }
        );
    }
    private:
    void receiveFromController(sensor_msgs::msg::Joy::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "I heard from ANGULAR: '%f' | LINEAR: '%f'", msg->axes[ANGULAR], msg->axes[LINEAR]);
        auto message = geometry_msgs::msg::TwistStamped();
        message.twist.angular.z = msg->axes[ANGULAR] * SPEED_OFFSET;
        message.twist.linear.x = msg->axes[LINEAR] * SPEED_OFFSET;
        this->cmd_publisher_->publish(message);
    };
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_publisher_;
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