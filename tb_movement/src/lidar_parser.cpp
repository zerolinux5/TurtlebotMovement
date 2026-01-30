#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tb_movement_msg/msg/obstacle.hpp"
#include "tb_movement_msg/msg/obstacle_scan.hpp"

constexpr int ANGULAR = 0;
constexpr int LINEAR = 1;
constexpr float SPEED_OFFSET = 2.0f;

struct Ray {
    float depth;
    float angle;
};

constexpr size_t MAX_RAYS = 720;

class LidarParser: public rclcpp::Node
{
    public:
    LidarParser()
    : Node("lidar_parser")
    {
        scan_publisher_ = this->create_publisher<tb_movement_msg::msg::ObstacleScan>("/obstacles", 10);
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, [this](sensor_msgs::msg::LaserScan::SharedPtr msg){ receiveFromLidar(msg); }
        );
        start_idx_ = 134;
    }
    private:
    void receiveFromLidar(sensor_msgs::msg::LaserScan::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Lidar data Angles: Min: %f Max: %f Increment: %f", msg->angle_min, msg->angle_max, msg->angle_increment);
        RCLCPP_INFO(this->get_logger(), "Lidar data: Time increment: %f Scan Time %f", msg->time_increment, msg->scan_time);
        RCLCPP_INFO(this->get_logger(), "Lidar data: Range: Min: %f Max: %f", msg->range_min, msg->range_max);
        RCLCPP_INFO(this->get_logger(), "Lidar data: First: %f Last: %f", msg->ranges[134], msg->ranges[226]);
        RCLCPP_INFO(this->get_logger(), "Lidar data: Ranges Size: %ld", msg->ranges.size());
        std::array<Ray, MAX_RAYS> buf;
        size_t n = 92;
        for (size_t idx = 0; idx < n; ++idx) {
            size_t idx_with_offset = start_idx_ + idx;
            buf[idx] = Ray{msg->ranges[idx_with_offset], msg->angle_increment * idx + msg->angle_min};
        }
        std::sort(buf.begin(), buf.begin() + n, [](const Ray& a, const Ray& b){ return a.depth < b.depth; });
        for (size_t idx = 0; idx < n; ++idx) {
            auto& ray = buf[idx];
            RCLCPP_INFO(this->get_logger(), "- Distance: %f Angle: %f", ray.depth, ray.angle);
        }
        tb_movement_msg::msg::ObstacleScan scan_msg;
        scan_msg.obstacles.reserve(n);

        for (size_t idx = 0; idx < n; ++idx) {
            tb_movement_msg::msg::Obstacle o;
            o.depth_m = buf[idx].depth;
            o.angle_rad = buf[idx].angle;
            scan_msg.obstacles.push_back(o);
        }
        scan_publisher_->publish(scan_msg);
    };
    rclcpp::Publisher<tb_movement_msg::msg::ObstacleScan>::SharedPtr scan_publisher_;
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