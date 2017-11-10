/*
 * NormalsEstimation.h
 *
 *  Created on: Oct 12, 2017
 *      Author: ivelas
 */

#include <but_velodyne/common.h>
#include <but_velodyne/GlobalOptimization.h>

#include <pcl/common/eigen.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#ifndef NORMALSESTIMATION_H_
#define NORMALSESTIMATION_H_

namespace but_velodyne {

void getPlaneCoefficients(const pcl::Normal &normal, const Eigen::Vector3f pt,
    float &a, float &b, float &c, float &d, bool normalize = true);

template <typename PointT>
void getNormals(const pcl::PointCloud<PointT> &subsampled_points,
    const pcl::PointCloud<PointT> &original_points,
    const std::vector<Origin> origins,
    const std::vector<Eigen::Affine3f> poses,
    const SensorsCalibration &calibration,
    const float radius,
    pcl::PointCloud<pcl::Normal> &normals) {

  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
  ne.setInputCloud(subsampled_points.makeShared());
  ne.setSearchSurface(original_points.makeShared());

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (radius);
  ne.compute(normals);

  for(int i = 0; i < subsampled_points.size(); i++) {
    pcl::Normal &n = normals[i];
    Eigen::Vector3f source = calibration.getSensorPose(poses[origins[i].pose_id], origins[i].sensor_id).translation();
    flipNormalTowardsViewpoint(subsampled_points[i], source(0), source(1), source(2),
        n.normal_x, n.normal_y, n.normal_z);
  }
}

} /* namespace but_velodyne */

#endif /* NORMALSESTIMATION_H_ */
