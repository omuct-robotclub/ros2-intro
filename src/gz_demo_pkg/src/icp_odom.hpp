#ifndef ICP_ODOM_HPP
#define ICP_ODOM_HPP

#include <Eigen/Dense>
#include <string>
#include <vector>

#include "ICP.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "laser_geometry/laser_geometry.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pcl/impl/point_types.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_representation.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace icp_odom {

class IcpOdom : public rclcpp::Node {
 public:
  IcpOdom(const rclcpp::NodeOptions& options);

 private:
  void scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  Eigen::Vector3d pos_ = Eigen::Vector3d::Zero();  // 位置、向き
  Eigen::Vector3d vel_ = Eigen::Vector3d::Zero();  // 速度、角速度

  pcl::PointCloud<pcl::PointXYZ>::Ptr pre_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  std::string base_frame_id_;
  std::string scan_topic_name_;

  ICP icp_dev;
  rclcpp::Time start_time_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace icp_odom

#endif  // ICP_ODOM_HPP