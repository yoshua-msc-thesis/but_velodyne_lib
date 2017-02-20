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

void loadLabels(const char *filename, vector<float> &labels) {
  ifstream fs(filename);
  float label;
  while (!fs.eof()) {
    fs >> label;
    labels.push_back(label);
  }
  cerr << "Loaded " << labels.size() << " labels from " << filename << endl;
}

void labelsByHeight(const VelodynePointCloud &cloud, float threshold, vector<float> &labels) {
  for (VelodynePointCloud::const_iterator p = cloud.begin(); p < cloud.end();
      p++) {
    if (p->y > threshold) {
      labels.push_back(1);
    } else {
      labels.push_back(0);
    }
  }
}

void colorizeCloudByLabels(const VelodynePointCloud &cloud, const vector<float> &labels,
    PointCloud<PointXYZRGB>::Ptr cloud_colored) {
  cloud_colored->resize(cloud.size());
  for(int i = 0; i < cloud.size(); i++) {
    PointXYZRGB &pt = cloud_colored->at(i);
    copyXYZ(cloud[i], pt);
    pt.r = pt.g = pt.b = 0;
    if(labels[i] < 0) {
      pt.r = pt.g = pt.b = 230;
    } else if(labels[i] > 0.5) {
      pt.r = 255;
    } else {
      pt.g = 200;
    }
  }
}

int main(int argc, char** argv) {
  if (argc < 4) {
    cerr << "Insufficient arguments. Usage: " << argv[0] << " <point-cloud> <height-threshold> <label-file>+" << endl;
    return EXIT_FAILURE;
  }

  VelodynePointCloud cloud;
  VelodynePointCloud::fromKitti(argv[1], cloud);
  cloud.setRingsByPointCount();
  cerr << "Loaded " << cloud.size() << " points from " << argv[1] << endl;

  vector<PointCloud<PointXYZRGB>::Ptr> clouds_to_show;
  for (int i = 2; i < argc; i++) {
    vector<float> labels;
    if (i == 2) {
      float height_thresh = atof(argv[2]);
      labelsByHeight(cloud, height_thresh, labels);
    } else {
      loadLabels(argv[i], labels);
    }

    PointCloud<PointXYZRGB>::Ptr cloud_colored(new PointCloud<PointXYZRGB>);
    colorizeCloudByLabels(cloud, labels, cloud_colored);
    clouds_to_show.push_back(cloud_colored);
  }

  Visualizer3D vis;
  float viewport_height = 1.0/clouds_to_show.size();
  for(int i = 0; i < clouds_to_show.size(); i++) {
    int viewport;
    vis.getViewer()->createViewPort(0.0, viewport_height*i, 1.0, viewport_height*(i+1), viewport);
    vis.getViewer()->setBackgroundColor(1.0, 1.0, 1.0, viewport);
    vis.addColorPointCloud(clouds_to_show[clouds_to_show.size() - i - 1], Eigen::Matrix4f::Identity(), viewport);
  }
  vis.show();

  return EXIT_SUCCESS;
}
