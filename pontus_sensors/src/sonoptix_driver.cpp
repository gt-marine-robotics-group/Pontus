#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/common/transforms.h>
#include "pcl_ros/transforms.hpp"
#include <pcl/filters/passthrough.h>

#include <curl/curl.h>
#include <sstream>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

class SonoptixDriverNode : public rclcpp::Node {

public:

  SonoptixDriverNode() : rclcpp::Node("sonoptix_driver") {
    // Parameters
    frame_id_ = this->declare_parameter<std::string>("frame_id", "");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "map");

    sonoptix_ip_addr_ = this->declare_parameter<std::string>("sonoptix_ip_addr", "192.168.1.211");
    sonar_range_ = this->declare_parameter<int>("sonar_range", 15);
    sonar_gain_ = this->declare_parameter<int>("sonar_gain", -20);
    sonar_res_m_ = this->declare_parameter<double>("sonar_res", 0.0148);
    sonar_angle_rad_ = this->declare_parameter<double>("sonar_angle", M_PI / 3.0);
    sonar_request_update_frequency_ms_ = this->declare_parameter<int>("sonar_freq_ms", 10);

    intensity_min_ = this->declare_parameter<int>("intensity_min", 1);
    normalize_intensity_ = this->declare_parameter<bool>("normalize_intensity", true);
    min_depth_m_ = this->declare_parameter<double>("min_depth_m", 0.2);   // keep points at least this deep
    max_depth_m_ = this->declare_parameter<double>("max_depth_m", 3.5);  // keep points at most this deep

    cluster_tolerance_ = this->declare_parameter<double>("cluster_tolerance", 0.15);
    cluster_min_points_ = this->declare_parameter<int>("cluster_min_points", 10);
    cluster_max_points_ = this->declare_parameter<int>("cluster_max_points", 200);

    rclcpp::QoS image_qos(rclcpp::KeepLast(1));
    image_qos.best_effort();

    configureSonoptix();
    // Read Sonoptix and generate point cloud at specified frequency
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(sonar_request_update_frequency_ms_),
      std::bind(&SonoptixDriverNode::publishPointCloud, this)
    );

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pontus/sonar/pointcloud", 10);
    clustered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pontus/sonar/clustercloud", 10);
    // img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    //   "/pontus/sonar_0/image_debug", image_qos, 
    //   std::bind(&SonoptixDriverNode::onImage, this, std::placeholders::_1)
    // );

    // TF buffer + listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(get_logger(),
      "Sonoptix: sonar_res=%.6f m/bin, sonar_angle=%.3f rad, thr=%d, norm=%s, frame_id='%s', target_frame='%s'",
      sonar_res_m_, sonar_angle_rad_, intensity_min_, normalize_intensity_ ? "true" : "false",
      frame_id_.c_str(), target_frame_.c_str()
    );
  }

private:
  void configureSonoptix() {
    // Configure Sonoptix 
    std::string api_url = "http://" + sonoptix_ip_addr_ + ":8000/api/v1";
    std::string body1 = std::string(R"({"enable": true, "sonar_range": )") + std::to_string(sonar_range_) + "}";
    long res_code1 = send_request(api_url + "/transponder", body1, "PATCH");

    std::string body2 = std::string(R"({"value": 2})");
    long res_code2 = send_request(api_url + "/streamtype", body2, "PUT");
    
    std::string body3 = std::string(R"({"contrast": 0, "gain": )") + std::to_string(sonar_gain_) + std::string(R"(, "mirror_image": false, "autodetect_orientation": 0)");
    long res_code3 = send_request(api_url + "/config", body3, "PATCH");

    if (res_code1 != 200 || res_code2 != 200 || res_code3 != 200) {
        RCLCPP_WARN(get_logger(),
                    "HTTP request for Sonoptix configuration failed");
    }

    // Open the RTSP connection
    std::string rtsp_url = "rtsp://" + sonoptix_ip_addr_ + ":8554/raw";
    cap_.open(rtsp_url, cv::CAP_FFMPEG);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
  }

  long send_request(const std::string& url, const std::string& json_body, const std::string& method) {
    CURL* curl = curl_easy_init();
    if (!curl) return -1;
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_body.c_str());
    curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 2000);
    curl_easy_setopt(curl, CURLOPT_CUSTOMREQUEST, method);
    CURLcode res = curl_easy_perform(curl);
    long status_code = -1;
    if (res == CURLE_OK) {
      curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &status_code);
    }
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    return status_code;
  }

  void publishPointCloud() {
    // Check if connection open
    if (!cap_.isOpened()) {
      RCLCPP_WARN(get_logger(), "Cannot read image because RTSP stream not open");
      return;
    }
    // Try to read frame
    cv::Mat frame;
    if (!cap_.read(frame)) {
      RCLCPP_WARN(get_logger(), "Failed to read frame from RTSP");
      return;
    }
    // Process frame and publish point cloud
    // I didn't end up using the onImage() method because it converts to cv::Mat and then calls sonar_image_to_cloud but I already have a cv::Mat
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = frame_id_;
    sensor_msgs::msg::PointCloud2 cloud_msg = sonar_image_to_cloud(frame, header);
    cloud_pub_->publish(cloud_msg);
  }

  // ----------- Image callback -----------
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat gray;
    if (!toGray(*msg, gray)) {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg = sonar_image_to_cloud(gray, msg->header);

    cloud_pub_->publish(cloud_msg);
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
      // last resort: ask for mono8
      out_gray = cv_bridge::toCvCopy(img, "mono8")->image;
      return true;
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_logger(), "cv_bridge conversion failed: %s", e.what());
      return false;
    }
  }

  sensor_msgs::msg::PointCloud2 sonar_image_to_cloud(
    const cv::Mat& gray,
    const std_msgs::msg::Header& in_header)
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;

    const int rows = gray.rows;
    const int cols = gray.cols;
    if (rows <= 0 || cols <= 0) {
      cloud_msg.header = in_header;
      if (!frame_id_.empty()) {
        cloud_msg.header.frame_id = frame_id_;
      }
      return cloud_msg;
    }
    
    const double angle_step = (cols > 0) ? (2.0 * sonar_angle_rad_ / static_cast<double>(cols)) : 0.0;
    const double angle_min  = -sonar_angle_rad_;

    const uint8_t threshold = static_cast<uint8_t>(std::clamp(intensity_min_, 0, 255));


    // Allocate PCL cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cloud->height = 1;  // This makes the pointcloud 2D instead of 3D
    cloud->is_dense = false;

    // Fill points
    for (int r = 0; r < rows; ++r) {
      const double d_m = sonar_res_m_ * static_cast<double>(r);
      const uint8_t* row_line = gray.ptr<uint8_t>(r);
      for (int c = 0; c < cols; ++c) {
        const uint8_t pixel_intensity = row_line[c];
        if (pixel_intensity <= threshold) {
          continue;
        }

        const double theta = angle_min + static_cast<double>(c) * angle_step;
        pcl::PointXYZI point;
        point.x = d_m * std::cos(theta);
        point.y = d_m * std::sin(theta);
        point.z = 0.0f;
        point.intensity = normalize_intensity_ ? (pixel_intensity) / 255.0f : (pixel_intensity);
        cloud->points.push_back(point);
      }
    }
    cloud->width = cloud->points.size();

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZI>);

    std_msgs::msg::Header src_header = in_header;
    if (!frame_id_.empty()) {
      src_header.frame_id = frame_id_;
    }

    // Try to transform to map frame
    const bool tf_ok = transform_pcl_to_map_frame(*cloud, src_header, *cloud_transformed);

    if (!tf_ok) {
      pcl::toROSMsg(*cloud, cloud_msg);
      cloud_msg.header = src_header;
      return cloud_msg;
    }

    // Filter in map frame
    *cloud_transformed = filter_points_pcl(*cloud_transformed, min_depth_m_, max_depth_m_);

    // Convert to ROS PointCloud2
    pcl::toROSMsg(*cloud_transformed, cloud_msg);
    cloud_msg.header = src_header;
    cloud_msg.header.frame_id = target_frame_;

    if(cloud_transformed->size() == 0 || cloud_transformed->size() != cloud_transformed->width * cloud_transformed->height) {
      return cloud_msg;
    }
    

    // cloud for clustering
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Euclidean clustering
    std::vector<pcl::PointIndices> clusters = euclideanClustering(cloud_transformed);
    std::vector<Eigen::Vector4f> centroids = computeClusterCentroids(cloud_transformed, clusters);

    // Allocate PCL cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    clustered_cloud->height = 1;
    clustered_cloud->is_dense = false;

    std::size_t num_points2 = 0;

    // Populate the occupied indices from each cluster
    for (const auto& centroid : centroids) {
        pcl::PointXYZI point;
        point.x = centroid[0];
        point.y = centroid[1];
        point.z = centroid[2];

        point.intensity = 100.0f;

        // these points are not transformed because they are based on an already transformed cloud
        clustered_cloud->points.push_back(point);
        num_points2++;

    }
    clustered_cloud->width = num_points2;

    // Convert to ROS PointCloud2
    sensor_msgs::msg::PointCloud2 clustered_msg;
    pcl::toROSMsg(*clustered_cloud, clustered_msg);
    clustered_msg.header = cloud_msg.header;

    // publish here, move outside function if desired
    if (clustered_msg.width > 1) {
        clustered_pub_->publish(clustered_msg);
    }

    return cloud_msg;
  }

  bool transform_pcl_to_map_frame(
    const pcl::PointCloud<pcl::PointXYZI>& cloud_in,
    const std_msgs::msg::Header& src_header,
    pcl::PointCloud<pcl::PointXYZI>& cloud_out)
  {
    if (src_header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "transform_pcl_to_map_frame(): empty frame_id; cannot transform to '%s'.",
                  target_frame_.c_str());
      return false;
    }

    try {
      // First, try at the cloud timestamp
      auto tf = tf_buffer_->lookupTransform(
        target_frame_,
        src_header.frame_id,
        rclcpp::Time(src_header.stamp),
        rclcpp::Duration::from_seconds(0.1)
      );

      pcl_ros::transformPointCloud(cloud_in, cloud_out, tf);
      return true;

    } catch (const tf2::ExtrapolationException& ex_future) {
      try {
        auto tf_latest = tf_buffer_->lookupTransform(
          target_frame_,
          src_header.frame_id,
          tf2::TimePointZero   // latest available transform
        );
        pcl_ros::transformPointCloud(cloud_in, cloud_out, tf_latest);

        RCLCPP_DEBUG(get_logger(),
          "TF future extrapolation for time %.3f; used latest transform instead.",
          rclcpp::Time(src_header.stamp).seconds());

        return true;

      } catch (const tf2::TransformException& ex2) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "TF fallback to latest also failed (%s -> %s): %s",
          src_header.frame_id.c_str(), target_frame_.c_str(), ex2.what());
        return false;
      }

    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF to '%s' failed from '%s': %s",
        target_frame_.c_str(), src_header.frame_id.c_str(), ex.what());
      return false;
    }
  }

  pcl::PointCloud<pcl::PointXYZI> filter_points_pcl(
    const pcl::PointCloud<pcl::PointXYZI>& cloud_in,
    double min_depth_m,
    double max_depth_m)
  {
    auto cloudPtr = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(cloud_in);
    auto filtered_data = new pcl::PointCloud<pcl::PointXYZI>();

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-max_depth_m, -min_depth_m); // Z axis is inverted from "depth"
    pass.filter(*filtered_data);

    return *filtered_data;
  }

  std::vector<pcl::PointIndices> euclideanClustering(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_data) {

      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
      tree->setInputCloud(pcl_data);

      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(cluster_tolerance_);
      ec.setMinClusterSize(cluster_min_points_); 
      ec.setMaxClusterSize(cluster_max_points_);
      ec.setSearchMethod(tree);
      ec.setInputCloud(pcl_data);
      ec.extract(cluster_indices);

      return cluster_indices;
  }

  std::vector<Eigen::Vector4f> computeClusterCentroids(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_data, const std::vector<pcl::PointIndices>& cluster_indices) {
      std::vector<Eigen::Vector4f> centroids;

      for (const auto& indices : cluster_indices) {
          Eigen::Vector4f centroid(0, 0, 0, 0);
          for (const auto& index : indices.indices) {
              centroid[0] += pcl_data->points[index].x;
              centroid[1] += pcl_data->points[index].y;
              centroid[2] += pcl_data->points[index].z;
          }
          centroid[0] /= indices.indices.size();
          centroid[1] /= indices.indices.size();
          centroid[2] /= indices.indices.size();
          centroid[3] = 1.0; // Homogeneous coordinate, usually set to 1
          
          centroids.push_back(centroid);
      }

      return centroids;
  }

private:
  // params
  std::string frame_id_;
  std::string target_frame_;

  std::string sonoptix_ip_addr_;
  int sonar_range_;
  int sonar_gain_;
  double sonar_res_m_;
  double sonar_angle_rad_;
  int sonar_request_update_frequency_ms_;

  int intensity_min_;
  bool normalize_intensity_;
  double min_depth_m_;
  double max_depth_m_;

  double cluster_tolerance_;
  int cluster_min_points_;
  int cluster_max_points_;

  // fields
  cv::VideoCapture cap_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr clustered_pub_;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonoptixDriverNode>());
  rclcpp::shutdown();
  return 0;
}
