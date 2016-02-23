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

using namespace std;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

int main(int argc, char** argv)
{
  if(argc < 2) {
    cerr << "Insufficient arguments. Usage: " << argv[0] << " [-p <poses>] <point-cloud>+";
    return 1;
  }

  vector<string> filenames;
  vector<Eigen::Affine3f> poses;
  for(int i = 1; i < argc; i++) {
    if(strcmp(argv[i], "-p") == 0 && (i < argc-1)) {
      i++;
      poses = KittiUtils::load_kitti_poses(argv[i]);
    } else {
      filenames.push_back(string(argv[i]));
    }
  }

  Visualizer3D visualizer;
  VelodynePointCloud cloud;
  for(int i = 0; i < filenames.size(); i++) {
    cerr << "scan: " << filenames[i] << endl;
    VelodynePointCloud::fromKitti(filenames[i], cloud);

    if(poses.empty()) {
      visualizer.addCloudColoredByHeight(cloud);
    } else {
      visualizer.addCloudColoredByHeight(cloud, poses[i].matrix());
    }
  }
  visualizer.show();

  return EXIT_SUCCESS;
}
