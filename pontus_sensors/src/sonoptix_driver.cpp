#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <chrono>

using namespace std::chrono_literals;

class SonoptixNode : public rclcpp::Node{
public:
    SonoptixNode() : Node("sonoptix_node") {
        auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pontus/sonar_0/points", qos_profile);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;
};

int main(int argc, char **argv)
{
   rclcpp::init(argc, argv);

   rclcpp::spin(std::make_shared<SonoptixNode>());

   rclcpp::shutdown();

   return 0;
}