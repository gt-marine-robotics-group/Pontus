#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

class SonoptixDriverNode : public rclcpp::Node {

public:

  SonoptixDriverNode() : rclcpp::Node("sonoptix_driver") {
    // Parameters
    frame_id_ = this->declare_parameter<std::string>("frame_id", "");
    sonar_res_m_ = this->declare_parameter<double>("sonar_res", 0.0148);
    sonar_angle_rad_ = this->declare_parameter<double>("sonar_angle", M_PI / 3.0);
    intensity_min_ = this->declare_parameter<int>("intensity_min", 1);
    normalize_intensity_ = this->declare_parameter<bool>("normalize_intensity", true);


    // this->declare_parameter("service_request_frequency", 1.0);
    // this->declare_parameter("z_axis_filter_limits", std::vector<double>{0.3, 1.5});
    // this->declare_parameter("x_axis_filter_limits", std::vector<double>{2.6, 100.0});
    // this->declare_parameter("cluster_tolerance", 0.15);
    // this->declare_parameter("cluster_min_points", 4);
    // this->declare_parameter("cluster_max_points", 200);

    // this->get_parameter("service_request_frequency", service_request_frequency_);
    // this->get_parameter("z_axis_filter_limits", z_axis_filter_limits_);
    // this->get_parameter("x_axis_filter_limits", x_axis_filter_limits_);
    // this->get_parameter("cluster_tolerance", cluster_tolerance_);
    // this->get_parameter("cluster_min_points", cluster_min_points_);
    // this->get_parameter("cluster_max_points", cluster_max_points_);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    qos.best_effort();

    cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pontus/sonar/pointcloud", qos);
    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/pontus/sonar_0/image_debug", qos, std::bind(&SonoptixDriverNode::onImage, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(),
      "Sonoptix: sonar_res=%.6f m/bin, sonar_angle=%.3f rad, thr=%d, norm=%s, frame_id='%s'",
      sonar_res_m_, sonar_angle_rad_, intensity_min_, normalize_intensity_ ? "true" : "false", frame_id_.c_str()
    );

    clustered_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pontus/sonar/clustercloud", qos);
  }

private:
  // ----------- Image callback -----------
  void onImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    cv::Mat gray;
    if (!toGray(*msg, gray)) {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg = sonar_image_to_cloud(gray, msg->header);

    // Check if pointcloud is valid
    if (cloud_msg.width > 1) {
        cloud_pub_->publish(cloud_msg);
    }
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

  sensor_msgs::msg::PointCloud2 sonar_image_to_cloud(const cv::Mat& gray,
                                                     const std_msgs::msg::Header& in_header)
  {
    sensor_msgs::msg::PointCloud2 out;

    const int rows = gray.rows;
    const int cols = gray.cols;
    if (rows <= 0 || cols <= 0) {
      out.header = in_header;
      if (!frame_id_.empty()) out.header.frame_id = frame_id_;
      return out;
    }

    const double angle_step = (cols > 0) ? (2.0 * sonar_angle_rad_ / static_cast<double>(cols)) : 0.0;
    const double angle_min  = -sonar_angle_rad_;

    const uint8_t threshold = static_cast<uint8_t>(std::clamp(intensity_min_, 0, 255));


    // Allocate PCL cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cloud->height = 1;
    cloud->is_dense = false;

    // Create XYZ cloud for clustering algorithm
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Fill points
    std::size_t num_points = 0;
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

        pcl::PointXYZ xyz_point; // XYZ points for euclidean clustering algorithm
        xyz_point.x = point.x
        xyz_point.y = point.y
        xyz_point.z = point.z

        filtered_cloud->points.push_back(xyz_point)

        point.intensity = normalize_intensity_
                         ? (pixel_intensity) / 255.0f
                         : (pixel_intensity);
        cloud->points.push_back(point);
        num_points++;
      }
    }

    cloud->width = num_points;

    // currently filtered_cloud uses the same points as cloud but in XYZ format, can add filter noise function here if desired

    filtered_cloud->width = num_points;


        // Euclidean clustering
        std::vector<pcl::PointIndices> clusters = euclideanClustering(filtered_cloud);
        std::vector<Eigen::Vector4f> centroids = computeClusterCentroids(filtered_cloud, clusters);

        // allocate new pcl cloud for clustered data
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = false;

        // initiate stuff
        std::vector<float> x_locations, y_locations;
        std::vector<float> values;

        // Populate the occupied indices from each cluster
        for (const auto& centroid : centroids) {
            geometry_msgs::msg::PointStamped point;
            point.header.stamp = transformed_msg.header.stamp;
            point.header.frame_id = base_frame_name;
            point.point.x = centroid[0];
            point.point.y = centroid[1];

            try {
                auto transformed_point = tf_buffer_->transform(point, "map");
                x_locations.push_back(transformed_point.point.x);
                y_locations.push_back(transformed_point.point.y);
                values.push_back(100.0); // Occupied
            } catch (const tf2::ExtrapolationException & ex) {
                RCLCPP_INFO(
                    this->get_logger(), "transform extrapolation failed: %s",
                    ex.what());
                return;
            } catch (const tf2::TransformException & ex) {
                RCLCPP_INFO(
                    this->get_logger(), "Could not transform map to %s: %s",
                    base_frame_name.c_str(), ex.what());
                return;
            }
        }

    // Convert to ROS PointCloud2
    pcl::toROSMsg(*cloud, out);
    out.header = in_header;
    if (!frame_id_.empty()) {
      out.header.frame_id = frame_id_;
    }

    // Convert to ROS PointCloud2
    pcl::toROSMsg(*clustered_cloud, clustered_ros);
    clustered_ros.header = in_header;
    if (!frame_id_.empty()) {
      clustered_ros.header.frame_id = frame_id_;
    }
    return out;
  }

    std::vector<pcl::PointIndices> euclideanClustering(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_data) {


        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
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

    std::vector<Eigen::Vector4f> computeClusterCentroids(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_data, const std::vector<pcl::PointIndices>& cluster_indices) {
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
  double sonar_res_m_;
  double sonar_angle_rad_;
  int intensity_min_;
  bool normalize_intensity_;
  std::string input_topic_;
  std::string output_topic_;

  // ROS I/O
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SonoptixDriverNode>());
  rclcpp::shutdown();
  return 0;
}