/*
 * Clustering.h
 *
 *  Created on: Oct 12, 2017
 *      Author: ivelas
 */

#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

#include <but_velodyne/point_types.h>

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
      const int K, std::vector<int> &indices, std::vector<float> &clusterProbs, const float train_ratio = 1.0f);

  void refineByKMeans(const pcl::PointCloud<PointT> &points, //const pcl::PointCloud<pcl::Normal> &normals,
      const int cluster_size, std::vector<int> &init_indices, std::vector<int> &refined_indices);

};

template<typename PointT>
void colorByClusters(const pcl::PointCloud<PointT> &in_cloud,
    const std::vector<int> &cluster_indices, const std::vector<float> &cluster_probs,
    pcl::PointCloud<LabeledPoint> &out_cloud) {
  out_cloud.resize(in_cloud.size());
  for(int i = 0; i < in_cloud.size(); i++) {
    copyXYZ(in_cloud[i], out_cloud[i]);
    out_cloud[i].intensity = in_cloud[i].intensity;
    out_cloud[i].label = cluster_indices[i];
    out_cloud[i].prob = cluster_probs[i];
  }
}

} /* namespace but_velodyne */

#endif /* CLUSTERING_H_ */
