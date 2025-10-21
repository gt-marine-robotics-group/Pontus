#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

// ------ Pointcloud Processor ------
class SonarPolarToRect {
public:
  struct Params {
    double sonar_res_m = 0.0148;        // meters per row (range bin)
    double sonar_angle_rad = M_PI / 3;  // half-FOV (+-60 degrees)
    int    intensity_min = 1;           // > threshold
    bool   normalize_intensity = true;  // [0..1] if true, else raw [0..255]
    std::string frame_id_override;      // optional
  };

  explicit SonarPolarToRect(const Params& p) : params_(p) {}

  sensor_msgs::msg::PointCloud2 makeCloud(const cv::Mat& gray, const std_msgs::msg::Header& hdr) const {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = hdr;
    if (!params_.frame_id_override.empty()) {
      cloud.header.frame_id = params_.frame_id_override;
    }

    const int rows = gray.rows;
    const int cols = gray.cols;

    if (rows <= 0 || cols <= 0) {
      cloud.height = 1; 
      cloud.width = 0; 
      cloud.is_dense = false; 
      cloud.is_bigendian = false;

      sensor_msgs::PointCloud2Modifier mod(cloud);
      mod.setPointCloud2Fields(
        4, 
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
      );
      return cloud;
    }

    const double angle_step = (cols > 0) ? (2.0 * params_.sonar_angle_rad / static_cast<double>(cols)) : 0.0;
    const double angle_min  = -params_.sonar_angle_rad;

    // Count points first
    const uint8_t thr = static_cast<uint8_t>(std::clamp(params_.intensity_min, 0, 255));
    std::size_t count = 0;
    for (int r = 0; r < rows; ++r) {
      const uint8_t* row = gray.ptr<uint8_t>(r);
      for (int c = 0; c < cols; ++c) if (row[c] > thr) ++count;
    }

    cloud.height = 1;
    cloud.width  = static_cast<uint32_t>(count);
    cloud.is_bigendian = false;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2Fields(
      4, 
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
    );
    mod.resize(count);

    sensor_msgs::PointCloud2Iterator<float> it_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> it_i(cloud, "intensity");

    std::size_t written = 0;
    for (int r = 0; r < rows; ++r) {
      const double d = params_.sonar_res_m * static_cast<double>(r);
      const uint8_t* row = gray.ptr<uint8_t>(r);
      for (int c = 0; c < cols; ++c) {
        const uint8_t pix = row[c];
        if (pix <= thr){
            continue;
        }

        const double theta = angle_min + static_cast<double>(c) * angle_step;
        *it_x = static_cast<float>(d * std::cos(theta));
        *it_y = static_cast<float>(d * std::sin(theta));
        *it_z = 0.0f;
        *it_i = params_.normalize_intensity ? (static_cast<float>(pix) / 255.0f)
                                       : static_cast<float>(pix);

        ++it_x; 
        ++it_y; 
        ++it_z; 
        ++it_i; 
        ++written;
      }
    }

    if (written != count) { 
        mod.resize(written); 
        cloud.width = static_cast<uint32_t>(written); 
    }

    return cloud;
  }

private:
  Params params_;
};


// ------ Synoptix Driver Node ------
class SynoptixDriverNode : public rclcpp::Node {
public:
  SynoptixDriverNode() : rclcpp::Node("synoptix_driver") {
    // Params
    frame_id_            = this->declare_parameter<std::string>("frame_id", "");
    sonar_res_m_         = this->declare_parameter<double>("sonar_res", 0.0148);
    sonar_angle_rad_     = this->declare_parameter<double>("sonar_angle", M_PI / 3.0);
    intensity_min_       = this->declare_parameter<int>("intensity_min", 1);
    normalize_intensity_ = this->declare_parameter<bool>("normalize_intensity", true);

    // QoS
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pontus/sonar/rect", qos);
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/pontus/sonar_0/image_debug", qos, std::bind(&SynoptixDriverNode::onImage, this, std::placeholders::_1));

    // Processor
    SonarPolarToRect::Params p;
    p.sonar_res_m = sonar_res_m_;
    p.sonar_angle_rad = sonar_angle_rad_;
    p.intensity_min = intensity_min_;
    p.normalize_intensity = normalize_intensity_;
    p.frame_id_override = frame_id_;
    proc_ = std::make_unique<SonarPolarToRect>(p);

    RCLCPP_INFO(get_logger(),
      "Sonar Pointcloud Params: sonar_res=%.6f m/bin, sonar_angle=%.3f rad, thr=%d, norm=%s, frame_id='%s'",
      input_topic_.c_str(), output_topic_.c_str(), sonar_res_m_, sonar_angle_rad_,
      intensity_min_, normalize_intensity_ ? "true" : "false", frame_id_.c_str());
  }

private:
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat gray;
    if (!toGray(*msg, gray)) {
        return;
    }
    auto cloud = proc_->makeCloud(gray, msg->header);
    cloud_pub_->publish(cloud);
  }

  bool toGray(const sensor_msgs::msg::Image& img, cv::Mat& out_gray) {
    try {
      if (img.encoding == "mono8") {
        out_gray = cv_bridge::toCvCopy(img, "mono8")->image;
        return true;
      }
      if (img.encoding == "bgr8" || img.encoding == "rgb8") {
        cv::Mat color = cv_bridge::toCvCopy(img, "bgr8")->image;
        cv::cvtColor(color, out_gray, cv::COLOR_BGR2GRAY);
        return true;
      }

      out_gray = cv_bridge::toCvCopy(img, "mono8")->image;
      return true;
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "cv_bridge conversion failed: %s", e.what());
      return false;
    }
  }

  // Params
  std::string frame_id_;
  double sonar_res_m_;
  double sonar_angle_rad_;
  int    intensity_min_;
  bool   normalize_intensity_;
  std::string input_topic_;
  std::string output_topic_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  // Processor
  std::unique_ptr<SonarPolarToRect> proc_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SynoptixDriverNode>());
  rclcpp::shutdown();
  return 0;
}
