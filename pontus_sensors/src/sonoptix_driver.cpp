#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

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

#include <Eigen/Core>

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
    sonar_res_m_ = this->declare_parameter<double>("sonar_res", 0.0148);
    sonar_angle_rad_ = this->declare_parameter<double>("sonar_angle", M_PI / 3.0);
    intensity_min_ = this->declare_parameter<int>("intensity_min", 1);
    normalize_intensity_ = this->declare_parameter<bool>("normalize_intensity", true);
    min_depth_m_ = this->declare_parameter<double>("min_depth_m", 0.2);   // keep points at least this deep
    max_depth_m_ = this->declare_parameter<double>("max_depth_m", 3.5);  // keep points at most this deep

    cluster_tolerance_ = this->declare_parameter<double>("cluster_tolerance", 0.15);
    cluster_min_points_ = this->declare_parameter<int>("cluster_min_points", 10);
    cluster_max_points_ = this->declare_parameter<int>("cluster_max_points", 200);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pontus/sonar/pointcloud", 10);
    clustered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pontus/sonar/clustercloud", 10);

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/pontus/sonar_0/image_debug", qos, 
      std::bind(&SonoptixDriverNode::onImage, this, std::placeholders::_1)
    );

    sim_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/pontus/sonar_0/sim",
      qos,
      std::bind(&SonoptixDriverNode::onSimPointcloud, this, std::placeholders::_1)
    );

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
  using CloudI = pcl::PointCloud<pcl::PointXYZI>;
  using CloudIPtr = CloudI::Ptr;

  // ----------- Callbacks -----------
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat gray;
    if (!toGray(*msg, gray)) {
      return;
    }

    CloudIPtr cloud = imageToPcl(gray);
    if(!cloud || cloud->empty()) {
      return;
    }

    processAndPublishCloud(cloud, msg->header);
  }

  void onSimPointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    CloudIPtr cloud = simPointcloudToPcl(*msg);
    if(!cloud || cloud->empty()) {
      return;
    }

    processAndPublishCloud(cloud, msg->header);
  }

  // ------ Front End Conversion --0----
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

  CloudIPtr imageToPcl(const cv::Mat& gray) {
    const int rows = gray.rows;
    const int cols = gray.cols;

    CloudIPtr cloud(new CloudI);
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.reserve(rows * cols);

    if (rows <= 0 || cols <= 0) {
      return cloud;
    }
    
    const double angle_step = (cols > 0) ? (2.0 * sonar_angle_rad_ / cols) : 0.0;
    const double angle_min  = -sonar_angle_rad_;

    const uint8_t threshold = static_cast<uint8_t>(std::clamp(intensity_min_, 0, 255));

    // Fill points
    for (int r = 0; r < rows; ++r) {
      const double d_m = sonar_res_m_ * static_cast<double>(r);
      const uint8_t* row_line = gray.ptr<uint8_t>(r);
      for (int c = 0; c < cols; ++c) {
        const uint8_t pixel_intensity = row_line[c];
        if (pixel_intensity <= threshold) {
          continue;
        }

        const double theta = angle_min + c * angle_step;
        pcl::PointXYZI point;
        point.x = d_m * std::cos(theta);
        point.y = d_m * std::sin(theta);
        point.z = 0.0f;
        point.intensity = normalize_intensity_ ? (pixel_intensity) / 255.0f : (pixel_intensity);
        cloud->points.push_back(point);
      }
    }
    cloud->width = cloud->points.size();

    return cloud;
}

  CloudIPtr simPointcloudToPcl(const sensor_msgs::msg::PointCloud2& msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::fromROSMsg(msg, cloud_xyz);

    CloudIPtr cloud(new CloudI);
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.reserve(cloud_xyz.points.size());

    for (const auto& p : cloud_xyz.points) {
      pcl::PointXYZI pi;
      pi.x = p.x;
      pi.y = p.y;
      pi.z = p.z;
      pi.intensity = 1.0f;
      cloud->points.push_back(pi);
    }

    cloud->width = cloud->points.size();
    return cloud;
  }

  // ------ Back End Filtering and Processing ------
  void processAndPublishCloud(const CloudIPtr& cloud_in,
                                 const std_msgs::msg::Header& in_header)
  {
    if (!cloud_in || cloud_in->empty()) {
      return;
    }

    std_msgs::msg::Header src_header = in_header;
    if (!frame_id_.empty()) {
      src_header.frame_id = frame_id_;
    }

    // Transform to target frame
    CloudIPtr cloud_transformed(new CloudI);
    bool tf_ok = transform_pcl_to_map_frame(*cloud_in, src_header, *cloud_transformed);

    sensor_msgs::msg::PointCloud2 cloud_msg;

    if (!tf_ok) {
      // publish in source frame as a fallback
      pcl::toROSMsg(*cloud_in, cloud_msg);
      cloud_msg.header = src_header;
      cloud_pub_->publish(cloud_msg);
      return;
    }

    // Filter in map frame
    CloudI cloud_filtered = filter_points_pcl(*cloud_transformed, min_depth_m_, max_depth_m_);

    // Convert to ROS PointCloud2
    pcl::toROSMsg(cloud_filtered, cloud_msg);
    cloud_msg.header = src_header;
    cloud_msg.header.frame_id = target_frame_;
    cloud_pub_->publish(cloud_msg);

    if (cloud_filtered.empty() || cloud_filtered.size() != cloud_filtered.width * cloud_filtered.height) {
      return;
    }

    // Clustering
    CloudIPtr cloud_filtered_ptr(new CloudI(cloud_filtered));

    // Euclidean clustering
    std::vector<pcl::PointIndices> clusters = euclideanClustering(cloud_filtered_ptr);
    std::vector<Eigen::Vector4f> centroids = computeClusterCentroids(cloud_filtered_ptr, clusters);

    // Allocate PCL cloud
    CloudIPtr clustered_cloud(new CloudI);
    clustered_cloud->height = 1;
    clustered_cloud->is_dense = false;

    // Populate the occupied indices for each cluster
    for (const auto& centroid : centroids) {
      pcl::PointXYZI point;
      point.x = centroid[0];
      point.y = centroid[1];
      point.z = centroid[2];

      point.intensity = 100.0f;
      clustered_cloud->points.push_back(point);
    }
    clustered_cloud->width = clustered_cloud->points.size();

    if (clustered_cloud->points.size() > 1) {
      sensor_msgs::msg::PointCloud2 clustered_msg;
      pcl::toROSMsg(*clustered_cloud, clustered_msg);
      clustered_msg.header = cloud_msg.header;
      clustered_pub_->publish(clustered_msg);
    }
  }

  bool transform_pcl_to_map_frame(
    const CloudI& cloud_in,
    const std_msgs::msg::Header& src_header,
    CloudI& cloud_out)
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

  CloudI filter_points_pcl(
    const pcl::PointCloud<pcl::PointXYZI>& cloud_in,
    double min_depth_m,
    double max_depth_m)
  {
    auto cloudPtr = std::make_shared<CloudI>(cloud_in);
    auto filtered_data = std::make_shared<CloudI>();

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloudPtr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(-max_depth_m, -min_depth_m); // Z axis is inverted from "depth"
    pass.filter(*filtered_data);

    return *filtered_data;
  }

  std::vector<pcl::PointIndices> euclideanClustering(const CloudIPtr& pcl_data) {

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

  std::vector<Eigen::Vector4f> computeClusterCentroids(const CloudIPtr& pcl_data, const std::vector<pcl::PointIndices>& cluster_indices) {
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
  double sonar_res_m_;
  double sonar_angle_rad_;
  int intensity_min_;
  bool normalize_intensity_;
  double min_depth_m_;
  double max_depth_m_;
  double cluster_tolerance_;
  int cluster_min_points_;
  int cluster_max_points_;

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sim_cloud_sub_;
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
