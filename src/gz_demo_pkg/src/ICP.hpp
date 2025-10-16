#ifndef ICP_HPP
#define ICP_HPP

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <Eigen/Dense>
#include <random>
#include <string>
#include <vector>

/*
ver-1.0
*/

// point to pointICPをするだけ

// x y yawを返す

class ICP {
 public:
  Eigen::Vector3d icp(const pcl::PointCloud<pcl::PointXYZ> &pre_cloud, const pcl::PointCloud<pcl::PointXYZ> &now_cloud, const int interation_num);  // icpを用いて2つの点群（ここでは一つ前と今）の間の変化量を出す。
 private:
  Eigen::Vector3d process(const pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, const Eigen::Vector3d now_par);  // ICPの処理部
  pcl::PointCloud<pcl::PointXYZ>::Ptr find_nearest_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr source, const pcl::PointCloud<pcl::PointXYZ>::Ptr target);          // 最近傍点を探す
  Eigen::VectorXd getResidual(const pcl::PointCloud<pcl::PointXYZ>::Ptr source, const pcl::PointCloud<pcl::PointXYZ>::Ptr target);                                      // 残差行列を求める
  Eigen::MatrixXd getJacobian(const Eigen::Vector3d &now_par, pcl::PointCloud<pcl::PointXYZ>::Ptr source);                                                              // ヤコビアンを求める

  Eigen::Matrix2d getR(const double &theta);
  void transformPointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr const cloud_in, const Eigen::Vector3d &par);
};

#endif