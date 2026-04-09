#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <dvl_msgs/msg/dvl.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/static_transform_broadcaster.hpp"

#include <pcl/common/transforms.h>
#include "pcl_ros/transforms.hpp"
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <limits>

class DVLProcessorNode : public rclcpp::Node {

public:

  DVLProcessorNode() : rclcpp::Node("dvl_processor") {
    // Parameters
    target_frame_ = this->declare_parameter<std::string>("target_frame", "map");
    // target_frame_ = this->declare_parameter<std::string>("target_frame", "base_link");

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();

    dvl_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/dvl/odometry", qos, 
      std::bind(&DVLProcessorNode::dvl_odom_callback, this, std::placeholders::_1)
    );

    dvl_data_sub_ = this->create_subscription<dvl_msgs::msg::DVL>(
      "/dvl/data", qos, 
      std::bind(&DVLProcessorNode::dvl_data_callback, this, std::placeholders::_1)
    );

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/pontus/odometry", 10);
    altimeter_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/altimeter/pointcloud", 10);
    beam_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/debug/beam/pointcloud", 10);

    // TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    /*
          ^
          |
         3 4 
         2 1
    */

    double beam_axis_angle = -22.5 * (M_PI / 180);
    // tf2::Quaternion beam_axis_rotation;
    // beam_axis_rotation.setRPY(, 0, 0); 

    // Beam angles
    for (u_int i = 0; i < 4; i++) {
      double angle = M_PI/4 + (i * M_PI / 2);
      tf2::Quaternion beam_number_rotation;
      beam_number_rotation.setRPY(0, (-M_PI/2 + beam_axis_angle), angle);

      geometry_msgs::msg::TransformStamped beam_tf;
      beam_tf.transform.rotation = tf2::toMsg(beam_number_rotation);
      // beam_tf.transform.rotation = tf2::toMsg(beam_axis_rotation * beam_number_rotation);

      std::cerr << beam_tf.transform.rotation.x << ", " 
                << beam_tf.transform.rotation.y << ", "
                << beam_tf.transform.rotation.z << ", "
                << beam_tf.transform.rotation.w << std::endl;

      beam_tfs.push_back(beam_tf);
    }

    // std::cerr << "tf size " << beam_tfs.size() << std::endl;
  }

private:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  using CloudPtr = Cloud::Ptr;

  // ----------- Callbacks -----------
  void dvl_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // std::cerr << "odom z: " << msg->pose.pose.position.z << std::endl;
  }

  void dvl_data_callback(const dvl_msgs::msg::DVL::SharedPtr msg) {
    std::cerr << "altitude: " << msg->altitude << std::endl;

    // BEAMS
    msg->header.frame_id = "dvl_a50_link";
    beam_cloud.clear(); // TODO: REMOVE THIS

    for (u_int i = 0; i < beam_tfs.size(); i++) {
      if (!msg->beams[i].valid) {
        continue;
      }

      // Create the source point
      geometry_msgs::msg::PointStamped beam_point;
      beam_point.header.frame_id = msg->header.frame_id;
      beam_point.header.stamp = this->get_clock()->now();
      // beam_point.point.x = 1.0;
      beam_point.point.x = msg->beams[i].distance;

      auto beam_tf = beam_tfs[i];

      geometry_msgs::msg::PointStamped transformed_beam_point;
      tf2::doTransform(beam_point, transformed_beam_point, beam_tf);

      geometry_msgs::msg::PointStamped map_point;

      try {
        auto tf = tf_buffer_->lookupTransform(
          target_frame_,
          msg->header.frame_id,
          rclcpp::Time(msg->header.stamp),
          rclcpp::Duration::from_seconds(0.1)
        );

        tf2::doTransform(transformed_beam_point, map_point, tf);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "TF to '%s' failed from '%s' at altimeter_cloud time %.3f: %s",
          target_frame_.c_str(),
          msg->header.frame_id.c_str(),
          rclcpp::Time(msg->header.stamp).seconds(),
          ex.what());
        return;
      }

      pcl::PointXYZ pcl_point(map_point.point.x, map_point.point.y, map_point.point.z);
      beam_cloud.points.push_back(pcl_point);
      beam_cloud.width = beam_cloud.points.size();
      beam_cloud.height = 1;

      std::cerr << map_point.point.x << ", " << map_point.point.y << ", " << map_point.point.z << std::endl;
    }

      sensor_msgs::msg::PointCloud2 beam_cloud_msg;
      pcl::toROSMsg(beam_cloud, beam_cloud_msg);
      beam_cloud_msg.header.frame_id = target_frame_;
      beam_cloud_msg.header.stamp = msg->header.stamp;
      beam_cloud_pub_->publish(beam_cloud_msg);

      // std::cerr << "beam_cloud size: " << beam_cloud.points.size() << std::endl;


    // ALTIMETER
    // altimeter_cloud.clear(); // TODO: REMOVE THIS

    // Create the source point
    geometry_msgs::msg::PointStamped altimeter_point;
    altimeter_point.header.frame_id = msg->header.frame_id;
    altimeter_point.header.stamp = this->get_clock()->now();
    altimeter_point.point.z = msg->altitude;

    geometry_msgs::msg::PointStamped map_point;

    try {
      auto tf = tf_buffer_->lookupTransform(
        target_frame_,
        msg->header.frame_id,
        rclcpp::Time(msg->header.stamp),
        rclcpp::Duration::from_seconds(0.1)
      );

      tf2::doTransform(altimeter_point, map_point, tf);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF to '%s' failed from '%s' at altimeter_cloud time %.3f: %s",
        target_frame_.c_str(),
        msg->header.frame_id.c_str(),
        rclcpp::Time(msg->header.stamp).seconds(),
        ex.what());
      return;
    }

    pcl::PointXYZ pcl_point(map_point.point.x, map_point.point.y, map_point.point.z);
    altimeter_cloud.points.push_back(pcl_point);
    altimeter_cloud.width = altimeter_cloud.points.size();
    altimeter_cloud.height = 1;

    sensor_msgs::msg::PointCloud2 altimeter_cloud_msg;
    pcl::toROSMsg(altimeter_cloud, altimeter_cloud_msg);
    altimeter_cloud_msg.header.frame_id = target_frame_;
    altimeter_cloud_msg.header.stamp = msg->header.stamp;
    altimeter_cloud_pub_->publish(altimeter_cloud_msg);

    // std::cerr << "altimeter_cloud size: " << altimeter_cloud.points.size() << std::endl;

    // fit_plane_pcl(altimeter_cloud + beam_cloud, map_point);
    if (altimeter_cloud.points.size() > 100) {
      fit_plane_pcl(altimeter_cloud, map_point);
    }
  }

  void fit_plane_pcl(const Cloud& cloud_in, geometry_msgs::msg::PointStamped altimeter_point)
  {
    CloudPtr altimeter_cloud(new Cloud(cloud_in));
    if (altimeter_cloud->empty()) {
      return;
    }

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.05);

    seg.setInputCloud (altimeter_cloud);
    seg.segment (*inliers, *coefficients);

    // std::cerr << "Plane Model coefficients: " << coefficients->values[0] << " " 
    //                                           << coefficients->values[1] << " "
    //                                           << coefficients->values[2] << " " 
    //                                           << coefficients->values[3] << std::endl;

    geometry_msgs::msg::TransformStamped t;
    
    // Set Header
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    // t.header.frame_id = "base_link";
    t.child_frame_id = "test_map";

    // Calculate Orientation
    Eigen::Vector3d normal(coefficients->values[0],
      coefficients->values[1],
      coefficients->values[2]
    );
    normal.normalize();
    
    // Find rotation from Z-up to the normal
    Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), normal);

    // t.transform.translation.x = altimeter_point.point.x;
    // t.transform.translation.y = altimeter_point.point.y;
    // t.transform.translation.z = altimeter_point.point.z;

    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
  }

  // params
  std::string target_frame_;

  Cloud altimeter_cloud;
  Cloud beam_cloud;

  // ROS I/O
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_odom_sub_;
  rclcpp::Subscription<dvl_msgs::msg::DVL>::SharedPtr dvl_data_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr altimeter_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr beam_cloud_pub_;

  // TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  std::vector<geometry_msgs::msg::TransformStamped> beam_tfs;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DVLProcessorNode>());
  rclcpp::shutdown();
  return 0;
}
