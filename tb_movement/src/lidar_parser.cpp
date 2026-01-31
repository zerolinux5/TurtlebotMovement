#include <memory>
#include <string>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tb_movement_msg/msg/obstacle.hpp"

constexpr int ANGULAR = 0;
constexpr int LINEAR = 1;
constexpr float SPEED_OFFSET = 2.0f;

class LidarParser: public rclcpp::Node
{
    public:
    LidarParser()
    : Node("lidar_parser")
    {
        scan_publisher_ = this->create_publisher<tb_movement_msg::msg::Obstacle>("/obstacles", 10);
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, [this](sensor_msgs::msg::LaserScan::SharedPtr msg){ receiveFromLidar(msg); }
        );
        start_idx_ = 134;
    }
    private:
    void receiveFromLidar(sensor_msgs::msg::LaserScan::SharedPtr msg) {
        size_t n = 92;
        size_t offset = 360;
        tb_movement_msg::msg::Obstacle scan_msg;
        float front_min = msg->ranges[start_idx_];
        float back_min = msg->ranges[start_idx_ + offset];
        for (size_t idx = 1; idx < n; ++idx) {
            size_t front_with_offset = start_idx_ + idx;
            size_t back_with_offset = start_idx_ + idx + offset;
            front_min = std::min<float>(front_min, msg->ranges[front_with_offset]);
            back_min = std::min<float>(back_min, msg->ranges[back_with_offset]);
        }
        RCLCPP_INFO(this->get_logger(), "Publishing Front Min: %f Back Min: %f", front_min, back_min);
        scan_msg.closest_front_m = front_min;
        scan_msg.closest_back_m = back_min;
        scan_publisher_->publish(scan_msg);
    };
    rclcpp::Publisher<tb_movement_msg::msg::Obstacle>::SharedPtr scan_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    size_t start_idx_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarParser>());
    rclcpp::shutdown();
    return 0;
}