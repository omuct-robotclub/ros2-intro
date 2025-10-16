#include "icp_odom.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rclcpp_components/register_node_macro.hpp>

namespace icp_odom {

double normalize_angle(double angle) {
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  } else if (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

IcpOdom::IcpOdom(const rclcpp::NodeOptions &options)
    : Node("icp_odom_node", options) {
  this->declare_parameter<std::string>("base_link", "base_link");
  this->declare_parameter<std::string>("scan_topic_name", "scan");
  base_frame_id_ = this->get_parameter("base_link").as_string();
  scan_topic_name_ = this->get_parameter("scan_topic_name").as_string();

  scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(scan_topic_name_, 10, std::bind(&IcpOdom::scan_callback, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  start_time_ = this->now();
}

void IcpOdom::scan_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (!pre_cloud_) {
    // 初回は pre_cloud_ を設定して終了
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloud);
    pre_cloud_ = cloud;
    return;
  }

  // PointCloud -> pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  // ICP
  Eigen::Vector3d delta_par = icp_dev.icp(*pre_cloud_, *cloud, 20);
  pre_cloud_ = cloud;

  // 外れ値チェック
  if (!std::isfinite(delta_par.x()) || !std::isfinite(delta_par.y()) || !std::isfinite(delta_par.z()))
    return;

  // 小さすぎるものは切り捨て
  if (delta_par.norm() < 10e-3) {
    delta_par = Eigen::Vector3d::Zero();
  }

  // 時間計算
  rclcpp::Time now = this->now();
  double dt = (now - start_time_).seconds();
  start_time_ = now;

  // ローパスフィルタをかけつつ速度を計算
  double alpha_vel = 0.0;
  vel_.x() = alpha_vel * vel_.x() + (1 - alpha_vel) * (delta_par.x() / dt);
  vel_.y() = alpha_vel * vel_.y() + (1 - alpha_vel) * (delta_par.y() / dt);
  vel_.z() = alpha_vel * vel_.z() + (1 - alpha_vel) * (delta_par.z() / dt);

  pos_.x() += delta_par.x() * cos(pos_.z()) - delta_par.y() * sin(pos_.z());
  pos_.y() += delta_par.x() * sin(pos_.z()) + delta_par.y() * cos(pos_.z());
  pos_.z() += delta_par.z();
  pos_.z() = normalize_angle(pos_.z());

  RCLCPP_INFO(this->get_logger(), "pos: x:%f y:%f r:%f", pos_.x(), pos_.y(), pos_.z());

  // Quaternion 作成
  tf2::Quaternion q;
  q.setRPY(0, 0, pos_.z());
  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();

  nav_msgs::msg::Odometry odom;
  geometry_msgs::msg::TransformStamped odom_tf;

  // TF
  odom_tf.header.stamp = msg->header.stamp;
  odom_tf.header.frame_id = "odom";
  odom_tf.child_frame_id = base_frame_id_;
  odom_tf.transform.translation.x = pos_.x();
  odom_tf.transform.translation.y = pos_.y();
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = q_msg;
  tf_broadcaster_->sendTransform(odom_tf);

  // Odometry
  odom.header.stamp = msg->header.stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id = base_frame_id_;
  odom.pose.pose.position.x = pos_.x();
  odom.pose.pose.position.y = pos_.y();
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = q_msg;
  odom.twist.twist.linear.x = vel_.x();
  odom.twist.twist.linear.y = vel_.y();
  odom.twist.twist.angular.z = vel_.z();
  odom_pub_->publish(odom);
}

}  // namespace icp_odom

RCLCPP_COMPONENTS_REGISTER_NODE(icp_odom::IcpOdom)
