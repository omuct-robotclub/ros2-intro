#include "ICP.hpp"

Eigen::Vector3d ICP::icp(const pcl::PointCloud<pcl::PointXYZ>& pre_cloud, const pcl::PointCloud<pcl::PointXYZ>& now_cloud, const int interation_num) {
  if (now_cloud.points.empty() || pre_cloud.points.empty()) {
    return Eigen::Vector3d::Zero();
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(now_cloud);           // 現在ICPされてる点群
  const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pre_cloud);  // 目標の点群
  Eigen::Vector3d delta_par(Eigen::Vector3d::Zero());                                                                    // 返す変化量（平行移動、z軸回転）
  for (int i = 0; i < interation_num; i++) {
    Eigen::Vector3d icp_par = process(src_cloud, target_cloud, delta_par);
    ICP::transformPointCloud2(src_cloud, icp_par);
    delta_par += icp_par;
  }
  return delta_par;
}

Eigen::Vector3d ICP::process(const pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, const Eigen::Vector3d now_par) {  // ICPの処理部
  Eigen::Vector3d delta_par;
  const pcl::PointCloud<pcl::PointXYZ>::Ptr nearest_points = find_nearest_points(src_cloud, target_cloud);  // nearest_cloud は now_cloud と同じ大きさ
  Eigen::VectorXd R = getResidual(src_cloud, nearest_points);
  Eigen::MatrixXd J = getJacobian(now_par, src_cloud);

  Eigen::MatrixXd Jt = J.transpose();
  Eigen::MatrixXd JtJ = Jt * J;
  Eigen::VectorXd JtR = Jt * R;
  delta_par = -JtJ.ldlt().solve(JtR);

  return delta_par;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ICP::find_nearest_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr source, const pcl::PointCloud<pcl::PointXYZ>::Ptr target) {  // 最近傍点を探す

  pcl::PointCloud<pcl::PointXYZ>::Ptr closest_points(new pcl::PointCloud<pcl::PointXYZ>);  // source各点に対応した最近傍点群
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  auto target_ptr = std::make_shared<const pcl::PointCloud<pcl::PointXYZ>>(*target);
  kdtree.setInputCloud(target_ptr);
  std::vector<int> closest_point_ID(1);
  std::vector<float> closest_point_dis(1);

  for (int i = 0; i < int(source->points.size()); i++) {
    kdtree.nearestKSearch(source->points[i], 1, closest_point_ID, closest_point_dis);
    closest_points->points.push_back(target->points[closest_point_ID[0]]);  // 最も近い点を蓄積していく
  }
  return closest_points;
}

Eigen::VectorXd ICP::getResidual(const pcl::PointCloud<pcl::PointXYZ>::Ptr source, const pcl::PointCloud<pcl::PointXYZ>::Ptr target) {  // 残差行列を求める

  Eigen::VectorXd R(2 * source->points.size());  // 残差行列
  for (int i = 0; i < int(source->points.size()); i++) {
    Eigen::Vector2d p(source->points[i].x, source->points[i].y);
    Eigen::Vector2d tp(target->points[i].x, target->points[i].y);
    Eigen::Vector2d ri = p - tp;
    R.segment<2>(2 * i) = ri;
  }
  return R;
}

Eigen::MatrixXd ICP::getJacobian(const Eigen::Vector3d& now_par, pcl::PointCloud<pcl::PointXYZ>::Ptr source) {  // ヤコビアンを求める
  size_t N = source->points.size();
  Eigen::MatrixXd J(2 * N, 3);  // 2Dヤコビ行列
  double theta = now_par.z();
  Eigen::Matrix2d r = getR(theta);
  for (size_t i = 0; i < N; i++) {
    Eigen::Vector2d p(source->points[i].x, source->points[i].y);
    Eigen::Matrix<double, 2, 3> Ji;
    Ji << 1, 0, -sin(theta) * p.x() - cos(theta) * p.y(),
        0, 1, cos(theta) * p.x() - sin(theta) * p.y();
    J.block<2, 3>(2 * i, 0) = Ji;
  }
  return J;
}

Eigen::Matrix2d ICP::getR(const double& theta) {
  Eigen::Matrix2d R;
  R << cos(theta), -sin(theta),
      sin(theta), cos(theta);
  return R;
}

void ICP::transformPointCloud2(pcl::PointCloud<pcl::PointXYZ>::Ptr const target_cloud, const Eigen::Vector3d& par) {
  float cos_theta = std::cos(par.z());
  float sin_theta = std::sin(par.z());

  for (size_t i = 0; i < target_cloud->points.size(); ++i) {
    float x = target_cloud->points[i].x;
    float y = target_cloud->points[i].y;

    // 回転
    float x_rot = cos_theta * x - sin_theta * y;
    float y_rot = sin_theta * x + cos_theta * y;

    // 平行移動
    target_cloud->points[i].x = x_rot + par.x();
    target_cloud->points[i].y = y_rot + par.y();
    target_cloud->points[i].z = target_cloud->points[i].z;  // zはそのまま
  }
}