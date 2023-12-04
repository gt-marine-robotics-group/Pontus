#include <bvt_sdk.h>
#include <pontus_msgs/msg/blueviewping.hpp>
#include <blueview_wrapper.hpp>
#include <chrono>
#include <functional>
#include <stdexcept>
#include <sstream>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/core/mat.hpp>
#include "opencv2/imgcodecs.hpp"

using namespace std::chrono_literals;

class BlueViewRosDriver: public rclcpp::Node {

public:
    BlueViewRosDriver();

    /// Loads blue view settings such as range from ROS params, see example launch file
    void initParams();

    /// Blocks until node is shutdown, generating pings and pushing them to ros
    void run();

private:
    //rclcpp::init(argc, argv);
    BlueViewSonar sonar;
    void loop();
    void get_ping();

    rclcpp::Node::SharedPtr node_handle_;
    image_transport::ImageTransport image_transport;
    image_transport::Publisher grayscale_pub, color_pub;
    rclcpp::Publisher<pontus_msgs::msg::Blueviewping>::SharedPtr raw_pub;
    cv_bridge::CvImagePtr grayscale_img, color_img;
    pontus_msgs::msg::Blueviewping::SharedPtr ping_msg;

    double period_seconds_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool do_grayscale, do_color, do_raw;
    std::string frame_id;
};
