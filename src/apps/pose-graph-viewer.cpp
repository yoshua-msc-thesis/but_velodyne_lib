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

#include <boost/program_options.hpp>

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
namespace po = boost::program_options;

int main(int argc, char** argv) {

  string line;
  vector<Eigen::Affine3f> poses;
  poses.push_back(Eigen::Affine3f::Identity());
  int expected_index = 1;
  Visualizer3D vis;
  cv::RNG &rng = cv::theRNG();
  while(getline(cin, line)) {
    int src_i, trg_i;
    float tx, ty, tz, rx, ry, rz;
    sscanf(line.c_str(), "EDGE3 %d %d %f %f %f %f %f %f", &src_i, &trg_i, &tx, &ty, &tz, &rx, &ry, &rz);
    printf("EDGE3 %d %d %f %f %f %f %f %f\n", src_i, trg_i, tx, ty, tz, rx, ry, rz);
    Eigen::Affine3f t = getTransformation(tx, ty, tz, rx, ry, rz);
    if(trg_i == expected_index && src_i == (expected_index-1)) {
      poses.push_back(poses.back()*t);
      expected_index++;
    } else {
      cerr << "Loop " << src_i << " -> " << trg_i << endl;
      if(src_i >= poses.size() || trg_i >= poses.size()) {
        cerr << "  WARGING: there are only " << poses.size() << " poses in graph - skipping" << endl;
      } else {
        const PointXYZ &src = KittiUtils::positionFromPose(poses[src_i]);
        PointXYZ src_transformed = KittiUtils::positionFromPose(poses[src_i]*t);
        const PointXYZ &trg = KittiUtils::positionFromPose(poses[trg_i]);
        PointCloudLine loop(src, trg);
        PointCloudLine expectation(src_transformed, trg);
        uchar r = rng(256);
        uchar g = rng(256);
        uchar b = rng(256);
        vis.setColor(b, g, r).addLine(loop).setColor(b, g, r).addArrow(expectation);
      }
    }
  }
  vis.addPosesDots(poses).show();

  return EXIT_SUCCESS;
}
