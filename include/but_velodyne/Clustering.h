/*
 * Clustering.h
 *
 *  Created on: Oct 12, 2017
 *      Author: ivelas
 */

#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

#include <cv.h>
#include <ml.h>

#ifndef CLUSTERING_H_
#define CLUSTERING_H_

namespace but_velodyne {

template <class PointT>
class Clustering {
public:

  void clusterKMeans(const pcl::PointCloud<PointT> &points, const pcl::PointCloud<pcl::Normal> &normals,
      const int K, std::vector<int> &indices);

  void clusterKMeans(const pcl::PointCloud<PointT> &points, const int K, std::vector<int> &indices);

  void clusterEM(const pcl::PointCloud<PointT> &points, const pcl::PointCloud<pcl::Normal> &normals,
      const int K, std::vector<int> &indices, std::vector<float> &clusterProbs);

  void refineByKMeans(const pcl::PointCloud<PointT> &points, //const pcl::PointCloud<pcl::Normal> &normals,
      const int cluster_size, std::vector<int> &init_indices, std::vector<int> &refined_indices);

};

} /* namespace but_velodyne */

#endif /* CLUSTERING_H_ */
