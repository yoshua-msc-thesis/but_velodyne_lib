/*
 * Detection of moving objects.
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 18/02/2016
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
#include <libgen.h>

#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include <boost/program_options.hpp>
#include <cv.h>

#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/EigenUtils.h>
#include <but_velodyne/LineCloud.h>
#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

#define log cerr

template <class PointType>
typename pcl::PointCloud<PointType>::Ptr downsampleCloud(
                typename pcl::PointCloud<PointType>::Ptr input,
                double resolution = 0.005f) {

    pcl::VoxelGrid<PointType> vg;
    typename pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>());
    vg.setInputCloud (input);
    vg.setLeafSize (resolution, resolution, resolution);
    vg.filter (*cloud_filtered);

    return cloud_filtered;
}

/**
 * ./move-detection cloud.bin
 */
int main(int argc, char** argv) {

  if (argc != 2)  {
    log << "Invalid arguments, expecting: " << argv[0] << " <cloud.bin>" << endl;
    return EXIT_FAILURE;
  }
  string filename(argv[1]);

  VelodynePointCloud original_cloud;
  log << "KITTI file: " << filename << endl << flush;
  if (filename.find(".pcd") != string::npos) {
    io::loadPCDFile(filename, original_cloud);
  } else {
    VelodynePointCloud::fromKitti(filename, original_cloud);
  }

  Visualizer3D vis;

  PolarGridOfClouds polar_grid(original_cloud);
  PointCloud<PointXYZ>::Ptr dense_cloud = LineCloud().generateDenseCloud(polar_grid, 110, 100, 400);

  //vis.keepOnlyClouds(0).addPointCloud(*dense_cloud).show();

  PointCloud<PointXYZ>::Ptr downsampled_cloud = downsampleCloud<pcl::PointXYZ>(dense_cloud, 0.1);

  vis.keepOnlyClouds(0).addPointCloud(*downsampled_cloud).show();

  return EXIT_SUCCESS;
}
