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

}
