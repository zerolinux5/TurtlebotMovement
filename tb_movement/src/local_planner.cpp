#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "tb_movement_msg/msg/obstacle.hpp"

constexpr float MIN_DIST = 0.5f;

class LocalPlanner: public rclcpp::Node
{
    public:
    LocalPlanner()
    : Node("local_planner"), is_front_clear_(false), is_back_clear_(false)
    {
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        obstacles_subscriber_ = this->create_subscription<tb_movement_msg::msg::Obstacle>(
            "/obstacles", 10, [this](tb_movement_msg::msg::Obstacle::SharedPtr msg){ receiveFromObstacles(msg); }
        );
        global_direction_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/raw_reading", 10, [this](geometry_msgs::msg::TwistStamped::SharedPtr msg){ receiveFromGlobal(msg); }
        );
    }
    private:
    void receiveFromObstacles(tb_movement_msg::msg::Obstacle::SharedPtr msg) {
        bool is_new_front_clear = true;
        if (msg->closest_front_m < MIN_DIST) {
            is_new_front_clear = false; 
        }
        is_front_clear_ = is_new_front_clear;
        bool is_new_back_clear = true;
        if (msg->closest_back_m < MIN_DIST) {
            is_new_back_clear = false; 
        }
        is_back_clear_ = is_new_back_clear;
    };

    void receiveFromGlobal(geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        auto message = geometry_msgs::msg::TwistStamped();
        message.twist.angular.z = msg->twist.angular.z;
        // Only move forward if front is clear
        if (msg->twist.linear.x > 0.0f && is_front_clear_) {
            message.twist.linear.x = msg->twist.linear.x;
        } else if (msg->twist.linear.x < 0.0f && is_back_clear_) {
            message.twist.linear.x = msg->twist.linear.x;
        }
        this->cmd_publisher_->publish(message);
    };

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr global_direction_;
    rclcpp::Subscription<tb_movement_msg::msg::Obstacle>::SharedPtr obstacles_subscriber_;
    bool is_front_clear_;
    bool is_back_clear_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalPlanner>());
    rclcpp::shutdown();
    return 0;
}