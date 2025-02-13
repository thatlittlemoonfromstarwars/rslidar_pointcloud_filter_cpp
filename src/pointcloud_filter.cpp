// pointcloud_filter.cpp
// Authored by: [Your Name]
// This node filters a LiDAR PointCloud2 message by removing extra rings
// and robot points, and then projects the result to a LaserScan.

#include <memory>
#include <vector>
#include <limits>
#include <cmath>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <pcl_conversions/pcl_conversions.h>  // for pcl::fromROSMsg / toROSMsg
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>               // for removeNaNFromPointCloud

class PointCloudFilter : public rclcpp::Node
{
public:
  PointCloudFilter()
  : Node("pointcloud_filter")
  {
    // Publishers for the filtered point cloud and LaserScan messages
    filtered_pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/rslidar_points_filtered", 10);
    laser_scan_pub_  = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    // Subscriber to the raw LiDAR point cloud
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/rslidar_points", 10,
      std::bind(&PointCloudFilter::pointCloudCallback, this, std::placeholders::_1));

    // Set filtering parameters (same as in the Python version)
    min_height_ = -0.335 - 0.2339 + 0.05;  // ~ -0.5189 m (ground level + 5cm)
    max_height_ = 5.0;                    // Maximum height

    // LaserScan parameters
    scan_angle_min_      = -M_PI;
    scan_angle_max_      =  M_PI;
    // 0.1-degree resolution: 0.1 deg = (π/180)/10 rad = π/1800 rad
    scan_angle_increment_ = M_PI / 180.0 / 10.0;
    scan_range_min_      = 0.2;
    scan_range_max_      = 200.0;

    // Number of bins for the LaserScan (full circle)
    num_bins_ = static_cast<int>((scan_angle_max_ - scan_angle_min_) / scan_angle_increment_);

    RCLCPP_INFO(this->get_logger(), "PointCloud filter node has started.");
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert the ROS2 PointCloud2 message to a PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *input_cloud);

    // ----- Step 1: Remove LiDAR rings (second returns) -----
    // The assumption is that the LiDAR produces 16 points per block, where every other block
    // corresponds to the second return (the unwanted ring). We discard odd-indexed blocks.
    const int block_size = 16;
    size_t total_points = input_cloud->points.size();
    size_t num_blocks = total_points / block_size;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
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
            // Only keep finite points
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z))
		{
		  filtered_cloud->points.push_back(pt);
		}
          }
        }
      }
    }

    // ----- Step 2: Apply Height Filtering -----
    // Remove points that fall outside the desired z-range (e.g., points from the robot itself)
    pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto & pt : filtered_cloud->points)
    {
      if (pt.z >= min_height_ && pt.z <= max_height_)
      {
        height_filtered_cloud->points.push_back(pt);
      }
    }
    height_filtered_cloud->header = filtered_cloud->header;

    // ----- Publish the Filtered 3D Point Cloud -----
    sensor_msgs::msg::PointCloud2 output_cloud_msg;
    pcl::toROSMsg(*filtered_cloud, output_cloud_msg);
    output_cloud_msg.header = msg->header;
    filtered_pc_pub_->publish(output_cloud_msg);

    // ----- Step 3: Create and Publish a 2D LaserScan Message -----
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header = msg->header;
    scan_msg.angle_min = scan_angle_min_;
    scan_msg.angle_max = scan_angle_max_;
    scan_msg.angle_increment = scan_angle_increment_;
    scan_msg.range_min = scan_range_min_;
    scan_msg.range_max = scan_range_max_;
    // Initialize the ranges vector with infinity.
    scan_msg.ranges.resize(num_bins_, std::numeric_limits<float>::infinity());

    // Convert each valid 3D point to a 2D polar coordinate (angle, range)
    for (const auto & pt : height_filtered_cloud->points)
    {
      float angle = std::atan2(pt.y, pt.x);
      float range = std::hypot(pt.x, pt.y);
      // Skip points that fall outside the acceptable range.
      if (range < scan_range_min_ || range > scan_range_max_)
      {
        continue;
      }
      // Compute the bin index for the given angle.
      int index = static_cast<int>((angle - scan_msg.angle_min) / scan_msg.angle_increment);
      if (index >= 0 && index < num_bins_)
      {
        // Update the bin with the smallest range seen so far.
        if (range < scan_msg.ranges[index])
        {
          scan_msg.ranges[index] = range;
        }
      }
    }
    laser_scan_pub_->publish(scan_msg);
  }

  // Member variables for ROS interfaces.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pc_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_pub_;

  // Filtering parameters.
  float min_height_;
  float max_height_;

  // LaserScan parameters.
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

