#include <memory>
#include <vector>
#include <limits>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <pcl_conversions/pcl_conversions.h>  
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>   

class PointCloudFilter : public rclcpp::Node
{
public:
  PointCloudFilter()
  : Node("pointcloud_filter")
  {
    // Publishers for filtered point cloud and LaserScan messages
    filtered_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rslidar_points_filtered", 10);
    laser_scan_pub_  = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    // Subscriber to the raw LiDAR point cloud
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points", 10,
      std::bind(&PointCloudFilter::pointCloudCallback, this, std::placeholders::_1));

    // Filtering parameters
    min_height_ = -0.335 - 0.2339 + 0.05;  // Minimum height to keep (ground level + 5cm)
    max_height_ = 5.0;                     // Maximum height

    // LaserScan parameters
    scan_angle_min_      = -M_PI;
    scan_angle_max_      =  M_PI;
    scan_angle_increment_ = M_PI / 180.0 / 10.0;  // 0.1-degree resolution
    scan_range_min_      = 0.2;
    scan_range_max_      = 200.0;

    num_bins_ = static_cast<int>((scan_angle_max_ - scan_angle_min_) / scan_angle_increment_);

    RCLCPP_INFO(this->get_logger(), "PointCloud filter node has started.");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert ROS2 PointCloud2 message to a PCL point cloud (preserving intensity)
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *input_cloud);

    // --- Step 1: Remove LiDAR Rings ---
    const int block_size = 16;  
    size_t total_points = input_cloud->points.size();
    size_t num_blocks = total_points / block_size;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    filtered_cloud->header = input_cloud->header;

    for (size_t i = 0; i < num_blocks; ++i)
    {
      if ((i % 2) == 0)  // Keep even-indexed blocks
      {
        for (int j = 0; j < block_size; ++j)
        {
          size_t idx = i * block_size + j;
          if (idx < input_cloud->points.size())
          {
            const auto & pt = input_cloud->points[idx];
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
            {
              filtered_cloud->points.push_back(pt);  // Preserve intensity
            }
          }
        }
      }
    }

    // --- Step 2: Apply Height Filtering ---
    pcl::PointCloud<pcl::PointXYZI>::Ptr height_filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto & pt : filtered_cloud->points)
    {
      if (pt.z >= min_height_ && pt.z <= max_height_)
      {
        height_filtered_cloud->points.push_back(pt);
      }
    }
    height_filtered_cloud->header = filtered_cloud->header;

    // --- Publish the Filtered 3D Point Cloud ---
    publishFilteredPointCloud(msg->header, height_filtered_cloud);

    // --- Step 3: Create and Publish a 2D LaserScan Message ---
    publishLaserScan(msg->header, height_filtered_cloud);
  }

  void publishFilteredPointCloud(const std_msgs::msg::Header &header, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header = header;
    filtered_pc_pub_->publish(cloud_msg);
  }

  void publishLaserScan(const std_msgs::msg::Header &header, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    if (cloud->points.empty())
      return;

    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header = header;
    scan_msg.angle_min = scan_angle_min_;
    scan_msg.angle_max = scan_angle_max_;
    scan_msg.angle_increment = scan_angle_increment_;
    scan_msg.range_min = scan_range_min_;
    scan_msg.range_max = scan_range_max_;
    scan_msg.ranges.resize(num_bins_, std::numeric_limits<float>::infinity());

    for (const auto & pt : cloud->points)
    {
      float angle = std::atan2(pt.y, pt.x);
      float range = std::hypot(pt.x, pt.y);

      if (range < scan_range_min_ || range > scan_range_max_)
        continue;

      int index = static_cast<int>((angle - scan_msg.angle_min) / scan_msg.angle_increment);
      if (index >= 0 && index < num_bins_)
      {
        scan_msg.ranges[index] = std::min(scan_msg.ranges[index], range);
      }
    }

    laser_scan_pub_->publish(scan_msg);
  }

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;

  // Filtering parameters
  float min_height_;
  float max_height_;

  // LaserScan parameters
  float scan_angle_min_;
  float scan_angle_max_;
  float scan_angle_increment_;
  float scan_range_min_;
  float scan_range_max_;
  int num_bins_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
