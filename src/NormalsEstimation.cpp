/*
 * NormalsEstimation.cpp
 *
 *  Created on: Oct 12, 2017
 *      Author: ivelas
 */

#include <but_velodyne/NormalsEstimation.h>

#include <pcl/point_traits.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <cv.h>

using namespace pcl;
using namespace std;

namespace but_velodyne {

void getPlaneCoefficients(const pcl::Normal &normal, const Eigen::Vector3f pt,
    float &a, float &b, float &c, float &d, bool normalize) {
  if(!normalize) {
    a = normal.normal_x;
    b = normal.normal_y;
    c = normal.normal_z;
    d = -normal.getNormalVector3fMap().dot(pt);
  } else {
    float n1 = normal.normal_x;
    float n2 = normal.normal_y;
    float n3 = normal.normal_z;
    float x1 = pt(0);
    float x2 = pt(1);
    float x3 = pt(2);
    float nx1 = n1*x1;
    float nx2 = n2*x2;
    float nx3 = n3*x3;

    float k = sqrt(sq(n1) + sq(n2) + sq(n3) + sq(nx1) + sq(nx2) + sq(nx3) + 2*nx1*(nx2+nx3) + 2*nx2*nx3);

    a = n1/k;
    b = n2/k;
    c = n3/k;

    d = -(a*x1 + b*x2 + c*x3);
  }
}

template<>
void getNormals<PointXYZI>(const PointCloud<PointXYZI> &subsampled_points,
    const PointCloud<PointXYZI> &original_points,
    const vector<int> origins,
    const PointCloud<PointXYZ> sensor_positions,
    pcl::PointCloud<pcl::Normal> &normals);

} /* namespace but_velodyne */
