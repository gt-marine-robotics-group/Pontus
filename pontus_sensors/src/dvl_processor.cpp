#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/Odometry.hpp>
#include <dvl_msgs/msg/DVLDR.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/static_transform_broadcaster.hpp"

#include <pcl/common/transforms.h>
#include "pcl_ros/transforms.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <limits>

class DVLProcessorNode : public rclcpp::Node {

public:

  DVLProcessorNode() : rclcpp::Node("dvl_processor") {
    // Parameters
    frame_id_ = this->declare_parameter<std::string>("frame_id", "");
    target_frame_ = this->declare_parameter<std::string>("target_frame", "map");

    min_depth_m_ = this->declare_parameter<double>("min_depth_m", 0.10);   // keep points at least this deep
    max_depth_m_ = this->declare_parameter<double>("max_depth_m", 2.3);  // keep points at most this deep

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();

    dvl_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/dvl/odometry", 1, 
      std::bind(&DVLProcessorNode::dvl_odom_callback, this, std::placeholders::_1)
    );

    dvl_data_sub_ = this->create_subscription<dvl_msgs::msg::DVLDR>(
      "/dvl/data", 1, 
      std::bind(&DVLProcessorNode::dvl_data_callback, this, std::placeholders::_1)
    );

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/pontus/odometry", 10);

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  }

private:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  using CloudPtr = Cloud::Ptr;

  // ----------- Callbacks -----------
  void dvl_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  }

  void dvl_data_callback(const dvl_msgs::msg::Data::SharedPtr msg) {
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

    const int min_row= std::max(
      0,
      static_cast<int>(std::ceil(min_dist_m_ / sonar_res_m_))
    );

    const int max_row = std::min(
      rows,
      static_cast<int>(std::floor(max_dist_m_ / sonar_res_m_)) + 1
    );

    // If the clipped range is empty, return an empty cloud.
    if (min_row >= max_row) {
      cloud->width = 0;
      return cloud;
    }

    for (int r = min_row; r < max_row; ++r) {
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
        point.intensity = normalize_intensity_
          ? static_cast<float>(pixel_intensity) / 255.0f
          : static_cast<float>(pixel_intensity);

        cloud->points.push_back(point);
      }
    }

    cloud->width = cloud->points.size();
    return cloud;
  }

  // ------ Back End Filtering and Processing ------
  void processAndPublishCloud(const CloudIPtr& cloud_in,
                            const std_msgs::msg::Header& in_header)
  {
    if (!cloud_in || cloud_in->empty()) {
      std_msgs::msg::Header hdr = in_header;
      if (!frame_id_.empty()) {
        hdr.frame_id = target_frame_;
      }
      publishEmptyCluster(hdr);
      return;
    }

    std_msgs::msg::Header src_header = in_header;
    if (!frame_id_.empty()) {
      src_header.frame_id = frame_id_;
    }

    if (timestamp_offset_ms_ != 0.0) {
      rclcpp::Time shifted_time(in_header.stamp);
      shifted_time = shifted_time - rclcpp::Duration::from_seconds(timestamp_offset_ms_ / 1000.0);
      src_header.stamp = shifted_time;
    }

    // Transform to target frame
    CloudIPtr cloud_transformed(new CloudI);
    bool tf_ok = transform_pcl_to_map_frame(*cloud_in, src_header, *cloud_transformed);

    sensor_msgs::msg::PointCloud2 cloud_msg;

    if (!tf_ok) {
      pcl::toROSMsg(*cloud_in, cloud_msg);
      cloud_msg.header = src_header;
      cloud_pub_->publish(cloud_msg);

      std_msgs::msg::Header cluster_header = in_header;
      cluster_header.frame_id = target_frame_;
      publishEmptyCluster(cluster_header);
      return;
    }

    // Filter in map frame
    CloudI cloud_filtered = filter_points_pcl(*cloud_transformed, min_depth_m_, max_depth_m_);
    cloud_filtered = filter_lines_pcl(cloud_filtered);

    // Publish filtered cloud even if empty
    pcl::toROSMsg(cloud_filtered, cloud_msg);
    cloud_msg.header = src_header;
    cloud_msg.header.frame_id = target_frame_;
    cloud_pub_->publish(cloud_msg);

    std::vector<Eigen::Vector4f> centroids;
    if (!cloud_filtered.empty()) {
      CloudIPtr cloud_filtered_ptr(new CloudI(cloud_filtered));
      std::vector<pcl::PointIndices> clusters = euclideanClustering(cloud_filtered_ptr);
      centroids = computeClusterCentroids(cloud_filtered_ptr, clusters);
    }

    CloudI clustered_cloud;
    clustered_cloud.height = 1;
    clustered_cloud.is_dense = true;
    clustered_cloud.points.reserve(centroids.size());

    for (const auto& centroid : centroids) {
      pcl::PointXYZI point;
      point.x = centroid[0];
      point.y = centroid[1];
      point.z = centroid[2];
      point.intensity = 100.0f;
      clustered_cloud.points.push_back(point);
    }
    clustered_cloud.width = clustered_cloud.points.size();

    sensor_msgs::msg::PointCloud2 clustered_msg;
    pcl::toROSMsg(clustered_cloud, clustered_msg);
    clustered_msg.header = cloud_msg.header;
    clustered_pub_->publish(clustered_msg);
  }

  bool transform_pcl_to_map_frame(
    const CloudI& cloud_in,
    const std_msgs::msg::Header& src_header,
    CloudI& cloud_out)
  {
    if (src_header.frame_id.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "transform_pcl_to_map_frame(): empty frame_id; cannot transform to '%s'.",
        target_frame_.c_str());
      return false;
    }

    try {
      // Only use the transform at the cloud timestamp.
      // If it is not available yet, fail rather than using the latest transform,
      // since using a mismatched transform time can cause map-frame drift while yawing.
      auto tf = tf_buffer_->lookupTransform(
        target_frame_,
        src_header.frame_id,
        rclcpp::Time(src_header.stamp),
        rclcpp::Duration::from_seconds(0.1)
      );

      pcl_ros::transformPointCloud(cloud_in, cloud_out, tf);
      return true;

    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF to '%s' failed from '%s' at cloud time %.3f: %s",
        target_frame_.c_str(),
        src_header.frame_id.c_str(),
        rclcpp::Time(src_header.stamp).seconds(),
        ex.what());
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

  CloudI filter_lines_pcl(const CloudI& cloud_in)
  {
    CloudIPtr cloud(new CloudI(cloud_in));
    if (cloud->empty()) {
      return *cloud;
    }

    pcl::SACSegmentation<pcl::PointXYZI> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(line_dist_threshold_);
    seg.setMaxIterations(500);

    pcl::ExtractIndices<pcl::PointXYZI> extract;

    while (true) {
      if (cloud->size() < static_cast<size_t>(line_min_inliers_)) {
        break;  // not enough points left to contain a “large” line
      }

      pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

      seg.setInputCloud(cloud);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.empty()) {
        // No line found
        break;
      }

      if (static_cast<int>(inliers->indices.size()) < line_min_inliers_) {
        // line exists but small, keep it
        break;
      }

      // Coeffs for line: point (x0, y0, z0), direction (dx, dy, dz)
      const double x0 = coefficients->values[0];
      const double y0 = coefficients->values[1];
      const double z0 = coefficients->values[2];
      const double dx = coefficients->values[3];
      const double dy = coefficients->values[4];
      const double dz = coefficients->values[5];

      Eigen::Vector3d p0(x0, y0, z0);
      Eigen::Vector3d dir(dx, dy, dz);
      const double dir_norm = dir.norm();
      if (dir_norm < 1e-6) {
        break;  // degenerate direction
      }
      Eigen::Vector3d dir_unit = dir / dir_norm;

      // Estimate line segment length along the direction
      double t_min = std::numeric_limits<double>::infinity();
      double t_max = -std::numeric_limits<double>::infinity();

      for (int idx : inliers->indices) {
        const auto& pt = cloud->points[idx];
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        double t = (p - p0).dot(dir_unit);  // projection scalar
        t_min = std::min(t_min, t);
        t_max = std::max(t_max, t);
      }

      double segment_length = (t_max - t_min);          // in “t” units
      double line_length_m  = std::fabs(segment_length); // since dir_unit is unit, t has meters

      if (line_length_m < line_min_length_m_) {
        // Long-enough line not found; treat remaining as non-wall
        break;
      }

      // Remove this line from the cloud
      extract.setInputCloud(cloud);
      extract.setIndices(inliers);
      extract.setNegative(true);  // keep everything except the line
      CloudIPtr cloud_without_line(new CloudI);
      extract.filter(*cloud_without_line);

      cloud.swap(cloud_without_line);

      // Loop to strip multiple walls if they exist
    }

    return *cloud;
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

  void publishEmptyCluster(const std_msgs::msg::Header& header)
  {
    CloudI empty_cloud;
    empty_cloud.height = 1;
    empty_cloud.width = 0;
    empty_cloud.is_dense = true;

    sensor_msgs::msg::PointCloud2 clustered_msg;
    pcl::toROSMsg(empty_cloud, clustered_msg);
    clustered_msg.header = header;
    clustered_pub_->publish(clustered_msg);
  }

private:
  // params
  std::string frame_id_;
  std::string target_frame_;

  // ROS I/O
  rclcpp::Subscriber<nav_msgs::msg::Odometry>::SharedPtr dvl_odom_sub_;
  rclcpp::Subscriber<dvl_msgs::msg::DVLDR>::SharedPtr dvl_data_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster>(this);

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DVLProcessorNode>());
  rclcpp::shutdown();
  return 0;
}
