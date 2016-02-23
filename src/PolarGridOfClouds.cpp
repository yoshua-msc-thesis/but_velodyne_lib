/*
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Martin Velas (ivelas@fit.vutbr.cz)
 * Supervised by: Michal Spanel & Adam Herout ({spanel|herout}@fit.vutbr.cz)
 * Date: 27/03/2015
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

#include <iostream>
#include <cassert>

#include <pcl/common/transforms.h>

#include <but_velodyne/PolarGridOfClouds.h>
#include <but_velodyne/Visualizer3D.h>

using namespace std;
using namespace pcl;

namespace but_velodyne {

int PolarGridOfClouds::POLAR_SUPERBINS = 36;
int PolarGridOfClouds::BIN_SUBDIVISION = 1;

std::ostream& operator<< (std::ostream &out, const CellId &id) {
  out << "[r:" << id.ring << "; p:" << id.polar << "]";
  return out;
}

PolarGridOfClouds::PolarGridOfClouds(
    const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &point_cloud) :
      polar_grid(getPolarBins()) {
  fill(point_cloud);
}

void PolarGridOfClouds::fill(
    const pcl::PointCloud<velodyne_pointcloud::PointXYZIR> &point_cloud) {
  for(VelodynePointCloud::const_iterator pt = point_cloud.begin();
        pt < point_cloud.end();
        pt++) {
      int ring = pt->ring;
      assert(ring < VelodynePointCloud::VELODYNE_RINGS_COUNT);
      assert(ring >= 0);

      int polar_bin = getPolarBinIndex(*pt);
      assert(polar_bin < getPolarBins());
      assert(polar_bin >= 0);

      polar_grid[polar_bin][ring].push_back(*pt);
    }
}

int PolarGridOfClouds::getPolarBinIndex(const velodyne_pointcloud::PointXYZIR &point) {
  static const float polar_bin_size = 360.0f / getPolarBins();
  float angle = getPolarAngle(point.x, point.z) + 180.00000001f;
  if(angle == 0.0) {
    return 0;
  }
  return floor(angle/polar_bin_size - 0.000000001);
}

void PolarGridOfClouds::showColored() {
  Visualizer3D visualizer;
  for(int polar = 0; polar < getPolarBins(); polar++) {
    for(int ring = 0; ring < VelodynePointCloud::VELODYNE_RINGS_COUNT; ring++) {
      visualizer.addPointCloud(polar_grid[polar][ring]);
    }
  }
  visualizer.show();
}

void PolarGridOfClouds::transform(const Eigen::Matrix4f &t) {
  for(int polar = 0; polar < getPolarBins(); polar++) {
    for(int ring = 0; ring < VelodynePointCloud::VELODYNE_RINGS_COUNT; ring++) {
      transformPointCloud(polar_grid[polar][ring], polar_grid[polar][ring], t);
    }
  }
}

}
