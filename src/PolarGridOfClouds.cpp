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
    const VelodynePointCloud &point_cloud, bool redistribute) :
      polar_grid(getPolarBins()) {
  fill(point_cloud, redistribute);
}

PolarGridOfClouds::PolarGridOfClouds() :
      polar_grid(getPolarBins()) {
}

void PolarGridOfClouds::fill(
    const VelodynePointCloud &point_cloud, bool redistribute) {
  std::vector<float> borders;
  if(redistribute) {
    borders = point_cloud.getMaxOfRingRanges();
  }
  for(VelodynePointCloud::const_iterator pt = point_cloud.begin();
        pt < point_cloud.end();
        pt++) {
    int ring;
    if(redistribute) {
      ring = computeNewRingIndex(*pt, borders);
    } else {
      ring = pt->ring;
    }
    assert(ring < VelodynePointCloud::VELODYNE_RINGS_COUNT);
    assert(ring >= 0);

    int polar_bin = getPolarBinIndex(*pt);
    assert(polar_bin < getPolarBins());
    assert(polar_bin >= 0);

    polar_grid[polar_bin][ring].push_back(*pt);
    indices.push_back(CellId(polar_bin, ring));
  }
}

int PolarGridOfClouds::computeNewRingIndex(const velodyne_pointcloud::PointXYZIR &point,
                                           const std::vector<float> &borders) {
  float range = compute2DRange(point);
  int new_ring_id;
  for(new_ring_id = VelodyneSpecification::RINGS-2; new_ring_id > 0; new_ring_id--) {
    if(borders[new_ring_id] > range) {
      break;
    }
  }
  return new_ring_id;
}

PolarGridOfClouds::Ptr PolarGridOfClouds::summarize() const {
  PolarGridOfClouds::Ptr sumarized = of(VelodynePointCloud());
  for(int polar = 0; polar < getPolarBins(); polar++) {
    for(int ring = 0; ring < VelodynePointCloud::VELODYNE_RINGS_COUNT; ring++) {
      if(!polar_grid[polar][ring].empty()) {
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(polar_grid[polar][ring], centroid);
        velodyne_pointcloud::PointXYZIR centroid_ir;
        centroid_ir.x = centroid(0);
        centroid_ir.y = centroid(1);
        centroid_ir.z = centroid(2);
        centroid_ir.ring = ring;
        centroid_ir.intensity = polar_grid[polar][ring].averageIntensity();
        sumarized->polar_grid[polar][ring].push_back(centroid_ir);
      }
    }
  }
  return sumarized;
}

void PolarGridOfClouds::showColored() {
  static Visualizer3D visualizer;
  visualizer.keepOnlyClouds(0);
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
