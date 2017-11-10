/*
 * GlobalOptimization.cpp
 *
 *  Created on: Sep 12, 2017
 *      Author: ivelas
 */

#include <but_velodyne/GlobalOptimization.h>

using namespace std;

namespace but_velodyne {

typedef pcl::PointXYZI PointType;

template<>
void cloud2matrix(const pcl::PointCloud<PointType> &cloud, Eigen::MatrixXf &matrix);

template<>
void matrix2cloud(const Eigen::MatrixXf &matrix, pcl::PointCloud<PointType> &cloud);

template<>
void reduceWeakestDimension(const pcl::PointCloud<PointType> &input, const Eigen::Matrix3f &covaraince,
    pcl::PointCloud<PointType> &reduced);

template<>
void getClosestMatches<PointType>(const pcl::PointCloud<PointType>::Ptr points, float query_ratio, std::vector<cv::DMatch> &matches);

void printPoseGraphPrefix(const std::vector<Eigen::Affine3f> &poses,
    float covariance_diagonal) {
  static const cv::Mat POSES_COVARIANCE = cv::Mat::eye(6, 6, CV_32FC1)*covariance_diagonal;
  for(int pi = 1; pi < poses.size(); pi++) {
    Eigen::Affine3f delta_pose = poses[pi-1].inverse() * poses[pi];
    std::cout << PoseGraphEdge(pi-1, pi, delta_pose.matrix(), POSES_COVARIANCE) << std::endl;
  }
}

}
