/*
 * Visualization of KITTI poses file.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 17/06/2015
 *
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdlib>
#include <cstdio>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/Visualizer3D.h>
#include <but_velodyne/KittiUtils.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/LineCloud.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

inline PointCloud<PointXYZI>& operator += (PointCloud<PointXYZI> &this_cloud, const VelodynePointCloud& other)
{
  // Make the resultant point cloud take the newest stamp
  if (other.header.stamp > this_cloud.header.stamp)
    this_cloud.header.stamp = other.header.stamp;

  size_t nr_points = this_cloud.points.size ();
  this_cloud.points.resize (nr_points + other.points.size ());
  for (size_t i = nr_points; i < this_cloud.points.size (); ++i) {
    copyXYZ(other.points[i - nr_points], this_cloud.points[i]);
    this_cloud.points[i].intensity = other.points[i - nr_points].intensity;
  }

  this_cloud.width    = static_cast<uint32_t>(this_cloud.points.size ());
  this_cloud.height   = 1;
  if (other.is_dense && this_cloud.is_dense)
    this_cloud.is_dense = true;
  else
    this_cloud.is_dense = false;
  return this_cloud;
}

void normalizeIntensity(const PointCloud<PointXYZI> &in, PointCloud<PointXYZI> &out) {
  out.resize(in.size());
  float mean = 0;
  for(int i = 0; i < in.size(); i++) {
    copyXYZ(in[i], out[i]);
    mean += in[i].intensity;
  }

  mean /= in.size();

  for(int i = 0; i < in.size(); i++) {
    if(mean < 0.5) {
      out[i].intensity = in[i].intensity/mean*0.5;
    } else {
      out[i].intensity = (in[i].intensity-mean)/(1-mean)*0.5 + 0.5;
    }
  }
}

int main(int argc, char** argv) {
  if (argc < 2) {
    cerr << "Insufficient arguments. Usage: " << argv[0] << " [-p <poses>] <point-cloud>+" << endl;
    return 1;
  }

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-p") == 0 && (i < argc - 1)) {
      i++;
      poses = KittiUtils::load_kitti_poses(argv[i]);
    } else {
      filenames.push_back(string(argv[i]));
    }
  }

  Visualizer3D visualizer;
  VelodynePointCloud cloud;
  PointCloud<PointXYZI> sum_cloud;
  for (int i = 0; i < filenames.size(); i++) {

    if(i%2 == 0) {
      continue;
    }

    std::cerr << "Processing KITTI file: " << filenames[i] << std::endl << std::flush;
    if (filenames[i].find(".pcd") != std::string::npos) {
      pcl::io::loadPCDFile(filenames[i], cloud);
      cloud.setImageLikeAxisFromKitti();
    } else {
      VelodynePointCloud::fromKitti(filenames[i], cloud);
    }

    if (!poses.empty()) {
      transformPointCloud(cloud, cloud, poses[i]);
      float x, y, z, rx, ry, rz;
      getTranslationAndEulerAngles(poses[i], x, y, z, rx, ry, rz);
    }
    sum_cloud += cloud;

    if (i % 100 == 0) {
      pcl::VoxelGrid<PointXYZI> grid;
      grid.setLeafSize(0.5, 0.5, 0.5);
      grid.setInputCloud(sum_cloud.makeShared());
      grid.filter(sum_cloud);
    }
  }
  normalizeIntensity(sum_cloud, sum_cloud);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud = Visualizer3D::colorizeCloud(sum_cloud, true);
  visualizer.addColorPointCloud(rgb_cloud).show();

  io::savePCDFileBinary("kitti_viewer_out.pcd", *rgb_cloud);

  return EXIT_SUCCESS;
}
