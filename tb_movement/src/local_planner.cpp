#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tb_movement_msg/msg/obstacle_scan.hpp"

constexpr float MIN_DIST = 0.5f;

class LocalPlanner: public rclcpp::Node
{
    public:
    LocalPlanner()
    : Node("local_planner"), is_front_clear_(false)
    {
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        obstacles_subscriber_ = this->create_subscription<tb_movement_msg::msg::ObstacleScan>(
            "/obstacles", 10, [this](tb_movement_msg::msg::ObstacleScan::SharedPtr msg){ receiveFromObstacles(msg); }
        );
        global_direction_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/raw_reading", 10, [this](geometry_msgs::msg::TwistStamped::SharedPtr msg){ receiveFromGlobal(msg); }
        );
    }
    private:
    void receiveFromObstacles(tb_movement_msg::msg::ObstacleScan::SharedPtr msg) {
        bool new_reading = true;
        for (const auto& obstacle: msg->obstacles) {
            if (obstacle.depth_m < MIN_DIST) {
                new_reading = false;
                break;
            }
        }
        is_front_clear_ = new_reading;
    };

    void receiveFromGlobal(geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        auto message = geometry_msgs::msg::TwistStamped();
        message.twist.angular.z = msg->twist.angular.z;
        // Only move forward if front is clear
        if (is_front_clear_) {
            message.twist.linear.x = msg->twist.linear.x;
        }
        this->cmd_publisher_->publish(message);
    };

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr global_direction_;
    rclcpp::Subscription<tb_movement_msg::msg::ObstacleScan>::SharedPtr obstacles_subscriber_;
    bool is_front_clear_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlanner>());
    rclcpp::shutdown();
    return 0;
}